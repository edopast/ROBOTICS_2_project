clear all
close all
              
%UGV_pos = [0; 0; 0]; % the ugv is the "world frame" for the AGV, but
%actually we should compute Pj points in a world frame (fixed)
P1 = [0.2; 0.15; 0.2]; %[m]
P2 = [0.2; -0.15; 0.2]; %[m]
P3 = [-0.2; -0.15; 0.2]; %[m]
P4 = [-0.2; 0.15; 0.2]; %[m]
P5 = [0.2; 0.15; 0.]; %[m]
P6 = [0.2; -0.15; 0.]; %[m]
P7 = [-0.2; -0.15; 0.]; %[m]
P8 = [-0.2; 0.15; 0]; %[m]
UGV = [0; 0; 0];

AGV1pos = [1 0 1];
AGV1rpy = [0 0 pi];

AGV2pos = [1 0 0];
AGV2rpy = [0 -pi/4 pi];

%% compute image plane points: what does CAMERA 1 see?
figure (1)
axis([-3.16*10^-3 3.16*10^-3  -2.37*10^-3  2.37*10^-3])
xlabel('y axis of image plane [m] -->') 
ylabel('x axis of image plane [m] -->')
[yn1,x1,y1] = computeCameraImage(AGV1pos,AGV1rpy,P1.');
[yn2,x2,y2] = computeCameraImage(AGV1pos,AGV1rpy,P2.');
[yn3,x3,y3] = computeCameraImage(AGV1pos,AGV1rpy,P3.');
[yn4,x4,y4] = computeCameraImage(AGV1pos,AGV1rpy,P4.');
[yn5,x5,y5] = computeCameraImage(AGV1pos,AGV1rpy,P5.');
[yn6,x6,y6] = computeCameraImage(AGV1pos,AGV1rpy,P6.');
[yn7,x7,y7] = computeCameraImage(AGV1pos,AGV1rpy,P7.');
[yn8,x8,y8] = computeCameraImage(AGV1pos,AGV1rpy,P8.');
[ynugv,xugv,yugv] = computeCameraImage(AGV1pos,AGV1rpy,UGV.');

if ynugv==1
    hold on
    plot(yugv,xugv,'xb','MarkerSize',8,'MarkerFaceColor','b')
    text(yugv,xugv,"ugv",'VerticalAlignment','top','HorizontalAlignment','center')
end
if yn1==1
    hold on
    plot(y1,x1,'or','MarkerSize',2.5,'MarkerFaceColor','r')
    text(y1,x1,num2str(1),'VerticalAlignment','top','HorizontalAlignment','center')
end
if yn2==1
    hold on
    plot(y2,x2,'og','MarkerSize',2.5,'MarkerFaceColor','g')
    text(y2,x2,num2str(2),'VerticalAlignment','top','HorizontalAlignment','center')
end
if yn3==1
    hold on
    plot(y3,x3,'ob','MarkerSize',2.5,'MarkerFaceColor','b')
    text(y3,x3,num2str(3),'VerticalAlignment','top','HorizontalAlignment','center')
end
if yn4==1
    hold on
    plot(y4,x4,'oy','MarkerSize',2.5,'MarkerFaceColor','y')
    text(y4,x4,num2str(4),'VerticalAlignment','top','HorizontalAlignment','center')
end

%% compute image plane points: what does CAMERA 2 see?
figure (2)
axis([-3.16*10^-3 3.16*10^-3  -2.37*10^-3  2.37*10^-3])
xlabel('y axis of image plane [m] -->') 
ylabel('x axis of image plane [m] -->')
[yn1_2,x1_2,y1_2] = computeCameraImage(AGV2pos,AGV2rpy,P1.');
[yn2_2,x2_2,y2_2] = computeCameraImage(AGV2pos,AGV2rpy,P2.');
[yn3_2,x3_2,y3_2] = computeCameraImage(AGV2pos,AGV2rpy,P3.');
[yn4_2,x4_2,y4_2] = computeCameraImage(AGV2pos,AGV2rpy,P4.');
[yn5_2,x5_2,y5_2] = computeCameraImage(AGV2pos,AGV2rpy,P5.');
[yn6_2,x6_2,y6_2] = computeCameraImage(AGV2pos,AGV2rpy,P6.');
[yn7_2,x7_2,y7_2] = computeCameraImage(AGV2pos,AGV2rpy,P7.');
[yn8_2,x8_2,y8_2] = computeCameraImage(AGV2pos,AGV2rpy,P8.');
[ynugv_2,xugv_2,yugv_2] = computeCameraImage(AGV2pos,AGV2rpy,UGV.');

if ynugv_2==1
    hold on
    plot(yugv_2,xugv_2,'xb','MarkerSize',8,'MarkerFaceColor','b')
    text(yugv_2,xugv_2,"ugv",'VerticalAlignment','top','HorizontalAlignment','center')
end
if yn1_2==1
    hold on
    plot(y1_2,x1_2,'or','MarkerSize',2.5,'MarkerFaceColor','r')
    text(y1_2,x1_2,num2str(1),'VerticalAlignment','top','HorizontalAlignment','center')
end
if yn2_2==1
    hold on
    plot(y2_2,x2_2,'og','MarkerSize',2.5,'MarkerFaceColor','g')
    text(y2_2,x2_2,num2str(2),'VerticalAlignment','top','HorizontalAlignment','center')
end
if yn3_2==1
    hold on
    plot(y3_2,x3_2,'ob','MarkerSize',2.5,'MarkerFaceColor','b')
    text(y3_2,x3_2,num2str(3),'VerticalAlignment','top','HorizontalAlignment','center')
end
if yn4_2==1
    hold on
    plot(y4_2,x4_2,'oy','MarkerSize',2.5,'MarkerFaceColor','y')
    text(y4_2,x4_2,num2str(4),'VerticalAlignment','top','HorizontalAlignment','center')
end

%% These 8 points in the 2 cameras are used to retrieve relative transform
% we use the matrix of intrinsics

K = [3.6*10^-3 0         0;
     0         3.6*10^-3 0;
     0         0         1];
 
%K_inv = inv(K(1:2,1:2));    % only first block of the inverse
K_inv=eye(2);
 
% so we need to "normalize" our points in the image plane

q1_1 = [y1+3.16*10^-3  2.37*10^-3-x1];
    
q2_1 = [y2+3.16*10^-3  2.37*10^-3-x2];

q3_1 = [y3+3.16*10^-3  2.37*10^-3-x3];
    
q4_1 = [y4+3.16*10^-3  2.37*10^-3-x4];
    
q5_1 = [y5+3.16*10^-3  2.37*10^-3-x5];
    
q6_1 = [y6+3.16*10^-3  2.37*10^-3-x6];
    
q7_1 = [y7+3.16*10^-3  2.37*10^-3-x7];
    
q8_1 = [y8+3.16*10^-3  2.37*10^-3-x8];
    
%--
    
q1_2 = [y1_2+3.16*10^-3  2.37*10^-3-x1_2];
    
q2_2 = [y2_2+3.16*10^-3  2.37*10^-3-x2_2];

q3_2 = [y3_2+3.16*10^-3  2.37*10^-3-x3_2];
    
q4_2 = [y4_2+3.16*10^-3  2.37*10^-3-x4_2];
    
q5_2 = [y5_2+3.16*10^-3  2.37*10^-3-x5_2];
    
q6_2 = [y6_2+3.16*10^-3  2.37*10^-3-x6_2];
    
q7_2 = [y7_2+3.16*10^-3  2.37*10^-3-x7_2];
    
q8_2 = [y8_2+3.16*10^-3  2.37*10^-3-x8_2];

matchedPoints1 = [q1_1;
                  q2_1;
                  q3_1;
                  q4_1;
                  q5_1;
                  q6_1;
                  q7_1;
                  q8_1];
              
matchedPoints2 = [q1_2;
                  q2_2;
                  q3_2;
                  q4_2;
                  q5_2;
                  q6_2;
                  q7_2;
                  q8_2];              
 
%% MATLAB fcns

cameraParams = cameraParameters("IntrinsicMatrix",[3.6*10^-3 0 0;
                                                   0 3.6*10^-3 0;
                                                   3.16*10^-3 2.37*10^-3 1], "WorldUnits", 'm');

E = estimateEssentialMatrix(matchedPoints1,matchedPoints2, cameraParams);

Rcam2drone = [0   -sin(pi/4) cos(pi/4);
              -1  0          0;
              0   -cos(pi/4) -sin(pi/4)];   % camera to drone

[relativeOrientation,relativeLocation] = relativeCameraPose(E,cameraParams,cameraParams,matchedPoints1,matchedPoints2);

ROTdronetwo2one= Rcam2drone*(relativeOrientation.')*(Rcam2drone.')

TRASLdronetwo2one= Rcam2drone*relativeLocation.'

