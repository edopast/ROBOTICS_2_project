function [InImagePlane,X_ImagePlane,Y_ImagePlane] = computeCameraImage(DronePos,DroneAng,UGVPos)
% COMPUTECAMERAIMAGE This function computes what a single drone can see
% (i.e. the point corresponding to the UGV on the image plane) and outputs
% it
% PARAMS:
% DronePos = The drone position wrt world frame (row vec)
% DroneAng = The drone RPY angles [rad] wrt world frame (row vec)
% UGVPos = UGV position (x,y,z) in the world frame (row vec)
% RETURNS:
% InImagePlane = TRUE/FALSE bool variable that says if the point is in the 
% real image plane, considering a frontal pinhole camera model
% X_ImagePlane,Y_ImagePlane = x and y coordinates on the image plane
% (centered at center of image plane), x pointing upwards, y pointing to
% the right
% Eg: [yn,x,y] = computeCameraImage([1.2 1.2 1.7],[0 0 pi+pi/6],[0 0 0]);

%% the camera is taken from paper 3D_Power_Line_Extraction_from_Multiple_Aerial_Imag
F = 3.6*10^-3;  % Focal length [m]
ImagePlaneSize = [6.32*10^-3  4.74*10^-3];  % Y x X [m  m]
PixelDensity = [4000  3000];  % Y x X - adimensional

% Camera position with respect to drone's CoM is arbitrary. It is oriented
% towards x axis of the drone. NOTE: cam = CoM in this code
X_camDrone = 0;  % [m]
Y_camDrone = 0;  % [m]
Z_camDrone = 0;  % [m]

% Camera orientation wrt drone ref frame (euler angles)
Tilt = -pi/4;  % [rad] -> the cam is pointing downwards 45°
%Pan = 0;  % [rad] -> the cam is not rotated wrt drone's z axis

% [def] rotation from cam frame to drone frame (just a simple rotation)
Rdrone_cam = [-sin(Tilt) 0  cos(Tilt);
              0         -1         0;
              cos(Tilt) 0  sin(Tilt)];

%% [def] TRANSFORMATION from drone frame to world frame (RPY)
R_yaw = [cos(DroneAng(3))   -sin(DroneAng(3))   0;
         sin(DroneAng(3))   cos(DroneAng(3))    0;
         0                  0                   1];
     
R_pitch =  [cos(DroneAng(2))   0                   sin(DroneAng(2));
            0                  1                   0;
            -sin(DroneAng(2))  0                   cos(DroneAng(2))];
        
R_roll = [1                  0                   0;
          0                  cos(DroneAng(1))    -sin(DroneAng(1));
          0                  sin(DroneAng(1))    cos(DroneAng(1))];
      
Rworld_drone = R_yaw*R_pitch*R_roll;
Tworld_drone = DronePos.';
%Transf_mat_world_drone = [Rworld_drone Tworld_drone;
%                          0   0   0    1];
Transf_mat_drone_world = [Rworld_drone.' -(Rworld_drone.')*Tworld_drone;
                          0    0    0     1];
                      
%% Transformation of UGV coordinates to drone frame, then to Camera frame
homog_coords = Transf_mat_drone_world*[UGVPos.'; 1];
UGV_drone_coords = homog_coords(1:3,1);   % ugv in drone reference frame

% camera
UGV_Camera_coords = (Rdrone_cam.')*UGV_drone_coords; % ugv in camera frame

%% To image plane (analogic)
x_imagePlane = F*UGV_Camera_coords(1)/UGV_Camera_coords(3); % x=f*X/Z
y_imagePlane = F*UGV_Camera_coords(2)/UGV_Camera_coords(3); % y=f*Y/Z

%% NON-IDEALITIES: pixels, add noise -> compute new x, y position on image plane
% we consider the first (0,0) pixel as the top-left point in image plane
% u <--> y, v <--> -x
v = -(PixelDensity(2)/ImagePlaneSize(2))*x_imagePlane + PixelDensity(2)/2;
u = (PixelDensity(1)/ImagePlaneSize(1))*y_imagePlane + PixelDensity(1)/2;

% gaussian noise on pixel (zero-mean, sigma=2)
 v = v + random('Normal',0,4);
 u = u + random('Normal',0,4);

% quantization (round to closest pixel)
v=round(v);
u=round(u);
%% check if the point is inside the image plane, frontal pinhole camera
if (v>=0) && (v<=2999) && (u>=0) && (u<=3999) && (UGV_Camera_coords(3) >= 0)
    InImagePlane = true;
       X_ImagePlane = -(v - PixelDensity(2)/2)*(ImagePlaneSize(2)/PixelDensity(2));
       Y_ImagePlane = (u - PixelDensity(1)/2)*(ImagePlaneSize(1)/PixelDensity(1));
%         X_ImagePlane = v;
%         Y_ImagePlane = u;
else
    InImagePlane = false;
    X_ImagePlane = 0;
    Y_ImagePlane = 0;
end

end

