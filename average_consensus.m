% this script has the aim to make a consensus average on the position of
% the UGV; the assumption is that the formation has been achieved and all
% the drones are correctly oriented, pointing towards the UGV
clear 
close all

%% initialize UGV struct

ugv.xw = 0;  ugv.yw = 0; ugv.hw = 0;    % robot position and orientation wrt world frame (UGV frame)


n=7;             % number of vertex of the regular polygon formation 
r=2;             % ray associated to the formatioon
x = ugv.xw;      % coordinate for the centre of the formation
y = ugv.yw;

% plotting the initial condition associated to the formation

%% plotting stuff...
convex_hull = zeros(n,2).';
point = zeros(2,1);

% using this formulation, i am assuming that there is an agent in
% coordinate (rho,0); all the other agents are disposed starting from this
% coordinate

for i = 0:n
       point(1) = x + r * cos(2 * pi * i / n);    
       point(2) = y + r * sin(2 * pi * i / n); 
       point = point.';
       convex_hull(1, i+1) = point(1);
       convex_hull(2, i+1) = point(2);  
end

% circumbscribed circle
ray = r;
xc = x;
yc = y;
theta = linspace(0,2*pi);
x = ray*cos(theta) + xc;
y = ray*sin(theta) + yc;

% convexhull definition given the vertix of the geometric pattern of the
% UAV formation
convex_hull = convex_hull.';
[k,av] = convhull(convex_hull);
plot(x,y)
hold on
plot(convex_hull(:,1),convex_hull(:,2),'*')
hold on
plot(ugv.xw, ugv.yw, 'p')
plot(convex_hull(k,1),convex_hull(k,2))


%% no, starting from the ideally described situation, the following code retrieve the 
% 2D position into the image plane of the point on the surface of the UGV, 
% that are successively used to compute the relative pose of the UAVs wrt
% UGV
 
close all 

% UGV point representation
yn = zeros(n,1);
x_result = zeros(n,1);
y_result = zeros(n,1);

% Point1 representation
point_1x = zeros(n,1);
point_1y = zeros(n,1);
point_1_bool = zeros(n,1);

% Point2 representation
point_2x = zeros(n,1);
point_2y = zeros(n,1);
point_2_bool = zeros(n,1);

% Point3 representation
point_3x = zeros(n,1);
point_3y = zeros(n,1);
point_3_bool = zeros(n,1);

% Point4 representation
point_4x = zeros(n,1);
point_4y = zeros(n,1);
point_4_bool = zeros(n,1);

%UGV_pos = [0; 0; 0]; % the ugv is the "world frame" for the AGV
% as initial condition we are considering that the UGV starts to move in
% the origin of the world reference frame

P1 = [0.2; 0.15; 0.2]; %[m] 
P2 = [0.2; -0.15; 0.2]; %[m]
P3 = [-0.2; -0.15; 0.2]; %[m]
P4 = [-0.2; 0.15; 0.2]; %[m]
height = 1.5;

for i = 1:n
    yaw_angle = pi+2*pi*(i-1)/n;
    [yn(i),x_result(i),y_result(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],[ugv.xw ugv.yw 0]);
    [point_1_bool(i),point_1x(i),point_1y(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],P1.');
    [point_2_bool(i),point_2x(i),point_2y(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],P2.');
    [point_3_bool(i),point_3x(i),point_3y(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],P3.');
    [point_4_bool(i),point_4x(i),point_4y(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],P4.');
end
 
%example: which UAV do you want to plot? (choose between 1 and n)
k = 7;
plotting_image_point(yn(k), point_1_bool(k), point_2_bool(k), point_3_bool(k), point_4_bool(k),x_result(k),y_result(k),point_1x(k),point_1y(k), point_2x(k),point_2y(k),point_3x(k),point_3y(k), point_4x(k),point_4y(k))

%% In this section, the pose of each drone is recovered
close all

droneLocation = cell(n,1);
droneOrientation = cell(n,1);
estimated_x = zeros(n,1);
estimated_y = zeros(n,1);
for i=1:n
    [droneLocation{i,1},droneOrientation{i,1}] = PoseReconstruction([point_1x(i) point_1y(i); point_2x(i) point_2y(i); point_3x(i) point_3y(i); point_4x(i) point_4y(i)]);
    estimated_x(i) = droneLocation{i,1}(1);
    estimated_y(i) = droneLocation{i,1}(2);
end

% plotting stuff
convex_hull_estimated = zeros(n,2).';
for i = 1:n
       convex_hull_estimated(1, i+1) = estimated_x(i);
       convex_hull_estimated(2, i+1) = estimated_y(i);  
end

convex_hull_estimated = convex_hull_estimated.';
[k,av] = convhull(convex_hull_estimated);

plot(convex_hull(:,1),convex_hull(:,2),'*')
hold on
plot(ugv.xw, ugv.yw, 'p')
plot(convex_hull(k,1),convex_hull(k,2))
hold on
plot(convex_hull_estimated(:,1),convex_hull_estimated(:,2),'*')
hold on
plot(convex_hull_estimated(k,1),convex_hull_estimated(k,2))

%% now, we move the UGV of 0.5 on x direction; y,z are still equivalent to zero 

close all
ugv.xw = 0.5;  ugv.yw = 0; ugv.zw = 0;

% since we are doing the same thing as before, after the motion of the UGV,
% we reuse the same variable; we change only the final result variable, 
% so as to distinguish the resulta to that obtained before

% since computerCameraImage want the position of the UGV wrt world 
% reference frame, we must consider the induced motion also for the points 
% used for the homography 

P1 = [0.2+ugv.xw; 0.15+ugv.yw; 0.2+ugv.zw]; %[m]
P2 = [0.2+ugv.xw; -0.15+ugv.yw; 0.2+ugv.zw]; %[m]
P3 = [-0.2+ugv.xw; -0.15+ugv.yw; 0.2+ugv.zw]; %[m]
P4 = [-0.2+ugv.xw; 0.15+ugv.yw; 0.2+ugv.zw]; %[m]
height = 1.5;

for i = 1:n
    yaw_angle = pi+2*pi*(i-1)/n;
    [yn(i),x_result(i),y_result(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],[ugv.xw ugv.yw 0]);
    [point_1_bool(i),point_1x(i),point_1y(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],P1.');
    [point_2_bool(i),point_2x(i),point_2y(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],P2.');
    [point_3_bool(i),point_3x(i),point_3y(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],P3.');
    [point_4_bool(i),point_4x(i),point_4y(i)] = computeCameraImage([convex_hull(i,1) convex_hull(i,2) height],[0 0 yaw_angle],P4.');
end

droneLocation2 = cell(n,1);
droneOrientation2 = cell(n,1);
estimated_x2 = zeros(n,1);
estimated_y2 = zeros(n,1);
for i=1:n
    [droneLocation2{i,1},droneOrientation2{i,1}] = PoseReconstruction([point_1x(i) point_1y(i); point_2x(i) point_2y(i); point_3x(i) point_3y(i); point_4x(i) point_4y(i)]);
    estimated_x2(i) = droneLocation2{i,1}(1);
    estimated_y2(i) = droneLocation2{i,1}(2);
end

x_variation = zeros(n,1);
y_variation = zeros(n,1);
for i=1:n
    x_variation(i) = estimated_x2(i) - estimated_x(i);
    y_variation(i) = estimated_y2(i) - estimated_y(i);
end

% x_variation, y_variation represent the variable that include 
% the motion difference that each UGV see in two different temporal instant

% x_variation
% y_variation

%% now, we define the graph in which construct the average consensus

% we recall here the graph of our network; in our case the sensor network
% consist in n different agent, where each agent is linked with its 
% two adiacent agent that are described in the formation

% initilize the nodes; the adjacency matrix is 
% A = adjacency matrix
close all
A = zeros(n,n);
for i = 1: n
    for j = 1: n
        if i == 1
            A(i,i+1) = 1;
            A(i, n) = 1;
        elseif i == n
            A(i,1) = 1;
            A(i,n-1) = 1;
        else
            A(i,i-1) = 1;
            A(i,i+1) = 1;
        end
    end
end

G = graph(A);
h = plot(G);
layout(h,'subspace')


% now we define the elements inside the "Metropolis matrix"
% for our graph, we always have that max(di, dj) = n, since all UAVs are
% linked with the UGV
%% now, we construct the consensus matrix with the metropolis definition

% here we are using off_diag_element = 1/(1+max(di,dj)), however
% since all the degree in the network are equal to two, then we set
% directly 2 instead of max(di, dj)

off_diag_element = 1/(1+2);

% W represent the weight of the metropolis matrix (average consensus matrix)
W = A;  % we start from the adjacency matrix for simplicity
W(W==1) = off_diag_element;
diag_element = zeros(n,1);

for i=1 : n
    diag_element(i) = 1-2/3; % from metropolis matrix inspection
end                          % for the same reason of (1/2) above;
                             % we always have (2/3) as sum on a row
for i=1:n
    W(i,i) = diag_element(i);
end

%% Consensus over x motion of the UGV

% we recall that our initial condition are defined in the variable
% x_variation and y_variation
n_iteration = 20; % number of iteration
consensus_average = cell(n_iteration,1);
temp_consensus = zeros(n,1);
last_consensus = x_variation; % setting the initial condition
% in this part, we are saving all the variables; totally inefficient
% this is the main reason to exploit iterative algorithm! 
% only for debugging purpose clearly
for i=1:n_iteration
    temp_consensus = W*last_consensus;
    consensus_average{i,1} = temp_consensus;
    last_consensus = temp_consensus;
end
summ = 0;
for i=1:n
    summ = summ + x_variation(i);
end
average = summ/n
consensus_average{n_iteration,1}

%% Consensus over y motion of the UGV

consensus_average = cell(n_iteration,1);
temp_consensus = zeros(n,1);
last_consensus = y_variation; % setting the initial condition

for i=1:n_iteration
    temp_consensus = W*last_consensus;
    consensus_average{i,1} = temp_consensus;
    last_consensus = temp_consensus;
end
summ = 0;
for i=1:n
    summ = summ + y_variation(i);
end
average = summ/n
consensus_average{n_iteration,1}






function plotting_image_point(bool_result, bool_point1, bool_point2, bool_point3, bool_point4, result_x, result_y, ...
    point1x, point1y, point2x, point2y, point3x, point3y,point4x, point4y)
    if bool_result && bool_point1 && bool_point2 && bool_point3 && bool_point4
        plot(result_y(1),result_x(1),'or','MarkerSize',3,'MarkerFaceColor','r')
        hold on 
        plot(point1y(1),point1x(1),'or','MarkerSize',3,'MarkerFaceColor','b')
        hold on 
        plot(point2y(1),point2x(1),'or','MarkerSize',3,'MarkerFaceColor','b')
        hold on 
        plot(point3y(1),point3x(1),'or','MarkerSize',3,'MarkerFaceColor','b')
        hold on 
        plot(point4y(1),point4x(1),'or','MarkerSize',3,'MarkerFaceColor','b')
        hold on 
        axis([-3.16*10^-3 3.16*10^-3  -2.37*10^-3  2.37*10^-3])
        title('image plane vision UAV')
    else
        disp("UGV is not seen by the image plane")
    end
end
