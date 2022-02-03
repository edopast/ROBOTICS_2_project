% this is a trial of the loop that the UAVs 
estimated_after.x = zeros(form_auv.n,1);
estimated_after.y = zeros(form_auv.n,1);

estimated_before.x = zeros(form_auv.n,1);
estimated_before.y = zeros(form_auv.n,1);

estimated_variation.x = zeros(form_auv.n,1);
estimated_variation.y = zeros(form_auv.n,1);

j = 1;
n_iteration = 20;

W = graph_setup(form_auv);
%waitforbuttonpress;

[estimated_before.x, estimated_before.y, convex_hull_estimated_before] = pose_estimation(ugv,form_auv,1);
plot_with_convexhull(convex_hull_estimated_before);
hold on 
plot_actual_formation(form_auv);
%waitforbuttonpress;
close all
plot_with_convexhull(convex_hull_estimated_before);
hold on
correct_formation(ugv,form_auv,j);
%waitforbuttonpress;





for i = 1:1
    j = j+2000;

    close all
    
    [estimated_after.x, estimated_after.y, convex_hull_estimated_after] = pose_estimation(ugv,form_auv,j);
    plot_with_convexhull(convex_hull_estimated_after);
    hold on 
    plot_actual_formation(form_auv);
    %waitforbuttonpress;
    close all
    plot_with_convexhull(convex_hull_estimated_after);
    hold on
    correct_formation(ugv,form_auv,j);
    %waitforbuttonpress;
    
    for k= 1:form_auv.n
        estimated_variation.x(k) = estimated_after.x(k) - estimated_before.x(k);
        estimated_variation.y(k) = estimated_after.y(k) - estimated_before.y(k);
    end 
    disp("before x");
    disp(estimated_before.x);
    disp("after x");
    disp(estimated_after.x);
    disp("before y");
    disp(estimated_before.y);
    disp("after y");
    disp(estimated_after.y);

    [consensus_average_x, average_x, last_consensus_x] = Consensus_iteration(n_iteration, estimated_variation.x, form_auv, W);
%     disp(last_consensus);
%     disp(ugv.trajectory.signals.values(1000,1));
%     disp(last_consensus_x);
%     disp("x");
%     disp(ugv.trajectory.signals.values(j,1));

    
    [consensus_average_y, average_y, last_consensus_y] = Consensus_iteration(n_iteration, estimated_variation.y, form_auv, W);
%     disp(last_consensus);
%     disp(ugv.trajectory.signals.values(1000,2));
%     disp(last_consensus_y);
%     disp("y");
%     disp(ugv.trajectory.signals.values(j,2));
    
    estimated_before.x = estimated_after.x;
    estimated_before.y = estimated_after.y;
   
end
close all 





