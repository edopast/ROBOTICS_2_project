function [droneLocation,droneOrientation] = PoseReconstruction(ImagePoints)
% POSERECONSTRUCTION This function retrieves the absolute pose of the AGV
% wrt the UGV reference frame. It uses 4 predefined points, so it
% implements P4P. ImagePoints is a 4x2 matrix: on each row, the [x y]
% coords in the image plane of the 4 points (order: 1;2;3;4). Remember that
% x points upwards, y rightwards. 
% Eg: ImagePoints=[x1 y1; x2 y2; etc..]
% [droneLocation,droneOrientation] = PoseReconstruction(ImagePoints)

%UGV_pos = [0; 0; 0]; % the ugv is the "world frame" for the AGV. The 4
%points we use are solidal with it:
P1 = [0.2; 0.15; 0.2]; %[m]
P2 = [0.2; -0.15; 0.2]; %[m]
P3 = [-0.2; -0.15; 0.2]; %[m]
P4 = [-0.2; 0.15; 0.2]; %[m]

% change image center and x,y axes
imagePoints_newcenter= [ImagePoints(1,2)+3.16*10^-3  2.37*10^-3-ImagePoints(1,1);
              ImagePoints(2,2)+3.16*10^-3  2.37*10^-3-ImagePoints(2,1);
              ImagePoints(3,2)+3.16*10^-3  2.37*10^-3-ImagePoints(3,1);
              ImagePoints(4,2)+3.16*10^-3  2.37*10^-3-ImagePoints(4,1)];
          
worldPoints= [P1.';
              P2.';
              P3.';
              P4.'];
          
%% use matlab functions (computer vision toolbox)

cameraParams = cameraParameters("IntrinsicMatrix",[3.6*10^-3 0 0;
                                                   0 3.6*10^-3 0;
                                                   3.16*10^-3 2.37*10^-3 1], "WorldUnits", 'm');
                            
[cameraOrientation,cameraLocation] = estimateWorldCameraPose(imagePoints_newcenter,worldPoints,cameraParams);

cameraOrientation = cameraOrientation.';    % Rotation matrix camera2world

%% go back to drone ref frame

Rcam2drone = [0   -sin(pi/4) cos(pi/4);
              -1  0          0;
              0   -cos(pi/4) -sin(pi/4)];   % camera to drone

droneOrientation = cameraOrientation*(Rcam2drone.'); % Rotation matrix drone2world

droneLocation = cameraLocation;  % row vector, translation drone wrt world

end

