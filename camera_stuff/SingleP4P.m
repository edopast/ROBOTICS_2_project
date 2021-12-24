function [g_matrix] = SingleP4P(P1_imPlane,P2_imPlane,P3_imPlane,P4_imPlane)
% SINGLEP4P This function Tries to retrieve the camera position in the
% world frame (UGV frame) from the view of 4 points with known displacement.
% PARAMS: 4 points (2x1) in the image plane, in the form [x; y]
% RETURNS: the transformation matrix g

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
          
%% coordinates of the points wrt world frame (UGV - [x; y; z])
%   P1       P2
%      \ x /
%      y o
%      /   \
%   P4       P3
P1 = [0.2; 0.15; 0.2]; %[m]
P2 = [0.2; -0.15; 0.2]; %[m]
P3 = [-0.2; -0.15; 0.2]; %[m]
P4 = [-0.2; 0.15; 0.2]; %[m]

%% Some matrices
K = [F  0  0;
     0  F  0;
     0  0  1];  % intrinsic param. matrix

PI_0 = [eye(3) zeros(3,1)]; % standard projection matrix

% g = [R T;
%      0 1]; % coordinate transformation from world to camera fr., in SE(3)
       
% P_matrix = K*PI_0*g = [F*R_11  F*R_12  F*R_13  F*T_x;
%                        F*R_21  F*R_22  F*R_23  F*T_y;
%                        R_31    R_32    R_33    T_z  ];   % TO BE FOUND!!!

%% Numeric Computation of g

z_row = zeros(1,3);

A_1 = [-P1.'   -1 z_row 0   (P1.')*P1_imPlane(1)   P1_imPlane(1);
       z_row   0  -P1.' -1  (P1.')*P1_imPlane(2)   P1_imPlane(2)];
A_2 = [-P2.'   -1 z_row 0   (P2.')*P2_imPlane(1)   P2_imPlane(1);
       z_row   0  -P2.' -1  (P2.')*P2_imPlane(2)   P2_imPlane(2)];
A_3 = [-P3.'   -1 z_row 0   (P3.')*P3_imPlane(1)   P3_imPlane(1);
       z_row   0  -P3.' -1  (P3.')*P3_imPlane(2)   P3_imPlane(2)];
A_4 = [-P4.'   -1 z_row 0   (P4.')*P4_imPlane(1)   P4_imPlane(1);
       z_row   0  -P4.' -1  (P4.')*P4_imPlane(2)   P4_imPlane(2)];
    
A = [A_1;
     A_2;
     A_3;
     A_4];
 
p_vec = null(A);
p_vec = p_vec(:,4);

R_1 = [p_vec(1)/F;
       p_vec(5)/F;
       p_vec(9);];  % col 1 of R

R_2 = [p_vec(2)/F;
       p_vec(6)/F;
       p_vec(10);];  % col 1 of R
   
R_3 = [p_vec(3)/F;
       p_vec(7)/F;
       p_vec(11);];  % col 1 of R
   
T = [p_vec(4)/F;
     p_vec(8)/F;
     p_vec(12);];
 
sigma = 1/norm(R_1);

g_matrix1 = sigma*[R_1 R_2 R_3 T;
            0   0   0   1/sigma]
        
g_matrix2 = -sigma*[R_1 R_2 R_3 T;
            0   0   0   -1/sigma]        
   
%% find svd decomposition and vectors associated to smaller singular values

% [U,S,V] = svd(A);
% 
% p_1 = V(:,9);
% p_2 = V(:,10);
% p_3 = V(:,11);
% p_4 = V(:,12);
% 
% "1"
% p_1(4)/F
% p_1(8)/F
% p_1(12)
% "2"
% p_2(4)/F
% p_2(8)/F
% p_2(12)
% "3"
% p_3(4)/F
% p_3(8)/F
% p_3(12)
% "4"
% p_4(4)/F
% p_4(8)/F
% p_4(12)

%%

% A_1 = [-P1.'   -1 z_row 0   (P1.')*P1_imPlane(1);
%        z_row   0  -P1.' -1  (P1.')*P1_imPlane(2)];
% A_2 = [-P2.'   -1 z_row 0   (P2.')*P2_imPlane(1);
%        z_row   0  -P2.' -1  (P2.')*P2_imPlane(2)];
% A_3 = [-P3.'   -1 z_row 0   (P3.')*P3_imPlane(1);
%        z_row   0  -P3.' -1  (P3.')*P3_imPlane(2)];
% A_4 = [-P4.'   -1 z_row 0   (P4.')*P4_imPlane(1);
%        z_row   0  -P4.' -1  (P4.')*P4_imPlane(2)];
%     
% A = [-A_1;
%      -A_2;
%      -A_3;
%      -A_4];
%  
% b = 2*[P1_imPlane(1);P1_imPlane(2);
%      P2_imPlane(1);P2_imPlane(2);
%      P3_imPlane(1);P3_imPlane(2);
%      P4_imPlane(1);P4_imPlane(2)];
% 
% 
% g_matrix = [pinv(A)*b ; 2]


%% check which possibility is the right one
 
%% returns



end

