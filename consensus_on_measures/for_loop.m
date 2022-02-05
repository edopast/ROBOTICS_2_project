%% trial of the loop to insert into the project outline 
estimated_after.x = zeros(form_auv.n,1);
estimated_after.y = zeros(form_auv.n,1);

estimated_before.x = zeros(form_auv.n,1);
estimated_before.y = zeros(form_auv.n,1);

estimated_variation.x = zeros(form_auv.n,1);
estimated_variation.y = zeros(form_auv.n,1);
yaw_error_vector = zeros(form_auv.n, 1);

formation_distance = zeros(2,25);
formation_error = zeros(1, 25);
index = 0;

%number of iteration for the consensus
n_iteration = 25;
%form_auv = form_auv_initial;

% creation of the consensus matrix; for further details open the function
% script 
W = graph_setup(form_auv);
close all;
%waitforbuttonpress;

%we run an estimation outside the loop so as to consider also the first
% variation induced by the motion of the UAV
[estimated_before.x, estimated_before.y, convex_hull_estimated_before] = pose_estimation(ugv,form_auv,1);

%%series of plot 
% plot_with_convexhull(convex_hull_estimated_before);
% hold on 
% plot_actual_formation(form_auv);
% waitforbuttonpress;
% close all
% plot_with_convexhull(convex_hull_estimated_before);
% hold on
% correct_formation(ugv,form_auv,j);
%waitforbuttonpress;
UAV_pursuit_trajectory = cell(25, 2);

% the loop do only one iteration (for now)
for i = 1:25
    % we observe an iteration after 2000 steps of the UGV 
    j = i*1600;

%     plot_with_convexhull(convex_hull_estimated_before);
%     hold on 
%     correct_formation(ugv, form_auv,j);
%     waitforbuttonpress;
%     close all 

    %close all
    [estimated_after.x, estimated_after.y,... 
        convex_hull_estimated_after, points_image_plane_y] = pose_estimation(ugv,form_auv,j);

    
    yaw_error_vector = yaw_error(points_image_plane_y, form_auv);
    
% series of possible plot
%     plot_with_convexhull(convex_hull_estimated_after);
%     hold on 
%     plot_actual_formation(form_auv);
%     waitforbuttonpress;
%     close all
%     plot_with_convexhull(convex_hull_estimated_after);
%     hold on
%     correct_formation(ugv,form_auv,j);
%     waitforbuttonpress;
%     close all;
    
    for k= 1:form_auv.n
        estimated_variation.x(k) = estimated_after.x(k) - estimated_before.x(k);
        estimated_variation.y(k) = estimated_after.y(k) - estimated_before.y(k);
    end 

    [consensus_average_x, average_x, last_consensus_x] = Consensus_iteration(n_iteration, ...
        estimated_variation.x, form_auv, W);
    [consensus_average_y, average_y, last_consensus_y] = Consensus_iteration(n_iteration, ...
        estimated_variation.y, form_auv, W);
    
    index = index + 1;
   % implementation of the motion after the estimation 
    for k= 1:form_auv.n
        UAV_pursuit_trajectory{index,1} = form_auv.xw;
        UAV_pursuit_trajectory{index,2} = form_auv.yw;
        form_auv.xw(k) = form_auv.xw(k) - last_consensus_x(k);
        form_auv.yw(k) = form_auv.yw(k) - last_consensus_y(k);
    end 

    
    [temp_x, temp_y] = formation_centre(form_auv);

    formation_distance(1,index) = ugv.trajectory.signals.values(j,1) - temp_x;
    formation_distance(2,index) = ugv.trajectory.signals.values(j,2) - temp_y;
    formation_error(index) = sqrt((ugv.trajectory.signals.values(j,1) - temp_x)^2 ...
    +(ugv.trajectory.signals.values(j,2) - temp_y)^2);

%      plot_with_convexhull(convex_hull_estimated_before);
%      hold on 
%      correct_formation(ugv, form_auv,j);
%      waitforbuttonpress;
%      close all 

    % last consensus_x, lat_consensus_y are the final estimation
    % of vector in both x,y direction starting from UAVs measruements 

    % after getting the estimation, then the UAVs move in the estimated 
    % direction; we want only to reproduce the trajectory to follow

%     disp(last_consensus_x);
%     disp(last_consensus_y);

   
% update necessary to make the cycle to continously iterate 
    estimated_before.x = estimated_after.x;
    estimated_before.y = estimated_after.y;
   
end

plot(formation_error, '-x')
hold on
waitforbuttonpress;
close all

plot_actual_formation(form_auv);
hold on 
[temp_x, temp_y] = formation_centre(form_auv);
plot(temp_x, temp_y,'o');

hold on;
dist = sqrt((ugv.trajectory.signals.values(j,1) - temp_x)^2 ...
    +(ugv.trajectory.signals.values(j,2) - temp_y)^2);
disp(dist);
correct_formation(ugv, form_auv,j);
%waitforbuttonpress;
close all 


% plot that consider motion from initial condition to 1000 iteration motion
% of the UGV
convergence_UAV = zeros(n_iteration ,form_auv.n); 
for i = 1 : n_iteration
    for j = 1:form_auv.n
     convergence_UAV(i,j) = consensus_average_x{i}(j);
    end
end

for k = 1:form_auv.n
     plot(convergence_UAV(:,k), '-x')
     hold on
end
close all

waitforbuttonpress;
point_x = 0;
point_y = 0;
vector_of_points = zeros(2, form_auv.n);
for i=1:index
    for j=1:form_auv.n

        point_x = UAV_pursuit_trajectory{i,1}(j);
        point_y = UAV_pursuit_trajectory{i,2}(j);
        vector_of_points(1, j) = point_x;
        vector_of_points(2, j) = point_y;

    end

    plot_with_convexhull(vector_of_points)
    hold on
    waitforbuttonpress;
    
end

j = 1;
for i=1:50
    j = i*800;
    hold on
    plot(ugv.trajectory.signals.values(j,1),ugv.trajectory.signals.values(j,2), '-x');
end





 






