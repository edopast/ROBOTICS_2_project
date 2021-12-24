clear all
close all
              
%UGV_pos = [0; 0; 0]; % the ugv is the "world frame" for the AGV, but
%actually we should compute Pj points in a world frame (fixed)
P1 = [0.2; 0.15; 0.2]; %[m]
P2 = [0.2; -0.15; 0.2]; %[m]
P3 = [-0.2; -0.15; 0.2]; %[m]
P4 = [-0.2; 0.15; 0.2]; %[m]
UGV = [0; 0; 0];

AGVpos = [0 -0.5 0.5];
AGVrpy = [0 0 pi/2];

%% compute image plane points: what does the camera see?
figure (1)
axis([-3.16*10^-3 3.16*10^-3  -2.37*10^-3  2.37*10^-3])
xlabel('y axis of image plane [m] -->') 
ylabel('x axis of image plane [m] -->')
[yn1,x1,y1] = computeCameraImage(AGVpos,AGVrpy,P1.');
[yn2,x2,y2] = computeCameraImage(AGVpos,AGVrpy,P2.');
[yn3,x3,y3] = computeCameraImage(AGVpos,AGVrpy,P3.');
[yn4,x4,y4] = computeCameraImage(AGVpos,AGVrpy,P4.');
[ynugv,xugv,yugv] = computeCameraImage(AGVpos,AGVrpy,UGV.');

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

%%

imagePoints= [y1+3.16*10^-3  2.37*10^-3-x1;
              y2+3.16*10^-3  2.37*10^-3-x2;
              y3+3.16*10^-3  2.37*10^-3-x3;
              y4+3.16*10^-3  2.37*10^-3-x4];
          
worldPoints= [P1.';
              P2.';
              P3.';
              P4.';];
          
cameraParams = cameraParameters("IntrinsicMatrix",[3.6*10^-3 0 0;
                                0 3.6*10^-3 0;
                                2.37*10^-3 3.16*10^-3 1], "WorldUnits", 'm');
                            
[cameraOrientation,cameraLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams);

% pcshow(worldPoints,'VerticalAxis','Y','VerticalAxisDir','down', ...
%      'MarkerSize',30);
% hold on
% plotCamera('Size',10,'Orientation',worldOrientation,'Location',...
%      worldLocation);
% hold off

%% go back to drone ref frame

droneLocation = cameraLocation  % row vector

droneOrientation = [0   -sin(pi/4) cos(pi/4);
                    -1  0          0;
                    0   -cos(pi/4) -sin(pi/4)]*cameraOrientation

