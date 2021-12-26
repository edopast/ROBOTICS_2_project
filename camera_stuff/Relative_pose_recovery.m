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

AGV1pos = [1 1 1.414];
AGV1rpy = [0 0 pi+pi/4];

AGV2pos = [0 1.414 1.414];
AGV2rpy = [0 0 pi+pi/2];

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
 
K_inv = inv(K(1:2,1:2));    % only first block of the inverse
 
% so we need to "normalize" our points in the image plane

q1_1 = [K_inv*[x1;
               y1];
        1];
    
q2_1 = [K_inv*[x2;
               y2];
        1];

q3_1 = [K_inv*[x3;
               y3];
        1];
    
q4_1 = [K_inv*[x4;
               y4];
        1];
    
q5_1 = [K_inv*[x5;
               y5];
        1];
    
q6_1 = [K_inv*[x6;
               y6];
        1];
    
q7_1 = [K_inv*[x7;
               y7];
        1];
    
q8_1 = [K_inv*[x8;
               y8];
        1];
    
%--
    
q1_2 = [K_inv*[x1_2;
               y1_2];
        1];
    
q2_2 = [K_inv*[x2_2;
               y2_2];
        1];

q3_2 = [K_inv*[x3_2;
               y3_2];
        1];
    
q4_2 = [K_inv*[x4_2;
               y4_2];
        1];
    
q5_2 = [K_inv*[x5_2;
               y5_2];
        1];
 
q6_2 = [K_inv*[x6_2;
               y6_2];
        1];
    
q7_2 = [K_inv*[x7_2;
               y7_2];
        1];
    
q8_2 = [K_inv*[x8_2;
               y8_2];
        1];   
%% Now, stack the kronecher products of all the points

Chi = [kron(q1_2,q1_1).';
       kron(q2_2,q2_1).';
       kron(q3_2,q3_1).';
       kron(q4_2,q4_1).';
       kron(q5_2,q5_1).';
       kron(q6_2,q6_1).';
       kron(q7_2,q7_1).';
       kron(q8_2,q8_1).'];
   
%% compute the possible E

poss_Es = null(Chi);

e1 = poss_Es(:,1);
e2 = poss_Es(:,2);
e3 = poss_Es(:,3);
e4 = poss_Es(:,4);

E_1=[e1(1:3).';
     e1(4:6).';
     e1(7:9).'];

E_2=[e2(1:3).';
     e2(4:6).';
     e2(7:9).'];

E_3=[e3(1:3).';
     e3(4:6).';
     e3(7:9).'];

E_4=[e4(1:3).';
     e4(4:6).';
     e4(7:9).'];
 
Rz_pihalf = [0  -1 0;
             1  0  0;
             0  0  1];
 
for j=1:4
   if j==1
        [U,S,V] = svd(E_1);
        T1=U(:,3)
        T2=-U(:,3)
        R1 = U*(Rz_pihalf)*(V.')
        R2 = U*(Rz_pihalf.')*(V.')
   elseif j==2
        [U,S,V] = svd(E_2);
        T1=U(:,3)
        T2=-U(:,3)
        R1 = U*(Rz_pihalf)*(V.')
        R2 = U*(Rz_pihalf.')*(V.')
   elseif j==3
        [U,S,V] = svd(E_3);
        T1=U(:,3)
        T2=-U(:,3)
        R1 = U*(Rz_pihalf)*(V.')
        R2 = U*(Rz_pihalf.')*(V.')
   elseif j==4
        [U,S,V] = svd(E_4);
        T1=U(:,3)
        T2=-U(:,3)
        R1 = U*(Rz_pihalf)*(V.')
        R2 = U*(Rz_pihalf.')*(V.')
   end
   
   newS11=0.5*(S(1,1)+S(2,2));
   newS22=0.5*(S(1,1)+S(2,2));
   S(1,1) = newS11;
   S(2,2) = newS22;
   S(3,3) = 0;
end
 
%% true E

[0 0 -0.999;
 0 0 -0.414;
 0.999 0.414 0]*[0.7 -0.7 0;
    0.7 0.7 0;
    0 0 1]

