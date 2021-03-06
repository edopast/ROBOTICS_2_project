function [estimated_x,estimated_y,convex_hull_estimated, points_image_plane_y] = pose_estimation(ugv,form_auv,t)
%% Function definition
% INPUT: 
%       - ugv: we exploit its trajectory geneerated initially;
%       - form_uav: actual behavior of the formation;
%       - t: time instant in which evaluate ugv trajectory

%% OUTPUT: 
%         - estimated_x: vector which elements are the estimation of UAVs
%                        present in the formation wrt axis x;
%         - estimated_y: vector which elements are the estimation of UAVs
%                        present in the foromation wrt axis y;
%         - convex_hull_estimated: convexhull constructed with estimated_x,
%                                  estimated_y; plotting reason


% the motion of the UGV is simulated at UAVs level (offset variable)
    x_offset = ugv.trajectory.signals.values(t,1);
    y_offset = ugv.trajectory.signals.values(t,2);
    heading  = ugv.trajectory.signals.values(t,3);
    
    ugv.xw = ugv.trajectory.signals.values(t,1);
    ugv.yw = ugv.trajectory.signals.values(t,2);
    
    form_ref_x= zeros(form_auv.n,1);
    form_ref_y= zeros(form_auv.n,1);
    form_ref_z= zeros(form_auv.n,1);
   
    for i = 1:form_auv.n
        form_ref_x(i,1) = form_auv.xw(i,1);%;-x_offset;
        form_ref_y(i,1) = form_auv.yw(i,1);%;-y_offset;
        form_ref_z(i,1) = form_auv.zw(i,1);
        
    end
  
    % UGV point representation (only for plotting purpose; the UAVs
    % does not know the position of the UGV)
    yn = zeros(form_auv.n,1);
    x_result = zeros(form_auv.n,1);
    y_result = zeros(form_auv.n,1);
    
    % Point1 representation
    point_1x = zeros(form_auv.n,1);
    point_1y = zeros(form_auv.n,1);
    point_1_bool = zeros(form_auv.n,1);
    
    % Point2 representation
    point_2x = zeros(form_auv.n,1);
    point_2y = zeros(form_auv.n,1);
    point_2_bool = zeros(form_auv.n,1);
    
    % Point3 representation
    point_3x = zeros(form_auv.n,1);
    point_3y = zeros(form_auv.n,1);
    point_3_bool = zeros(form_auv.n,1);
    
    % Point4 representation
    point_4x = zeros(form_auv.n,1);
    point_4y = zeros(form_auv.n,1);
    point_4_bool = zeros(form_auv.n,1);
    
    P1 = [ugv.P1(1)*cos(heading)-ugv.P1(2)*sin(heading);ugv.P1(1)*sin(heading) + ugv.P1(2)*cos(heading) ; ugv.P1(3)]; %[m]
    P2 = [ugv.P2(1)*cos(heading)-ugv.P2(2)*sin(heading);ugv.P2(1)*sin(heading) + ugv.P2(2)*cos(heading) ; ugv.P2(3)]; %[m]
    P3 = [ugv.P3(1)*cos(heading)-ugv.P3(2)*sin(heading);ugv.P3(1)*sin(heading) + ugv.P3(2)*cos(heading) ; ugv.P3(3)]; %[m]
    P4 = [ugv.P4(1)*cos(heading)-ugv.P4(2)*sin(heading);ugv.P4(1)*sin(heading) + ugv.P4(2)*cos(heading) ; ugv.P4(3)]; %[m]
    
    P1 = [P1(1)+x_offset; P1(2)+y_offset; ugv.P1(3)]; %[m]
    P2 = [P2(1)+x_offset; P2(2)+y_offset; ugv.P2(3)]; %[m]
    P3 = [P3(1)+x_offset; P3(2)+y_offset; ugv.P3(3)]; %[m]
    P4 = [P4(1)+x_offset; P4(2)+y_offset; ugv.P4(3)]; %[m]
    
    for i = 1:form_auv.n
        
        [yn(i),x_result(i),y_result(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollr(i,1) form_auv.pitchr(i,1) form_auv.yaww(i,1)], ...
            [x_offset y_offset 0]);
    
        [point_1_bool(i),point_1x(i),point_1y(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollr(i,1) form_auv.pitchr(i,1) form_auv.yaww(i,1)], P1.');
    
        [point_2_bool(i),point_2x(i),point_2y(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollr(i,1) form_auv.pitchr(i,1) form_auv.yaww(i,1)], P2.');
    
        [point_3_bool(i),point_3x(i),point_3y(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollr(i,1) form_auv.pitchr(i,1) form_auv.yaww(i,1)], P3.');
    
        [point_4_bool(i),point_4x(i),point_4y(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollr(i,1) form_auv.pitchr(i,1) form_auv.yaww(i,1)], P4.');
    
    end

    points_image_plane_y = cell(form_auv.n,1);

   
    for k = 1:form_auv.n
        points_image_plane_y{k} = [point_1y(k), point_2y(k),...
            point_3y(k), point_4y(k)];
    end

    
    % bool variable to check if all UAVs had the capability of see the UGV
    UGV_seen = 0;
    for i = 1:form_auv.n
        if yn(i) && point_1_bool(i) && point_2_bool(i) && point_3_bool(i) && point_4_bool(i)
            UGV_seen = UGV_seen+1;
        end

    end

    if(UGV_seen ~= form_auv.n)
        for i = 1:form_auv.n
            if yn(i) && point_1_bool(i) && point_2_bool(i) && point_3_bool(i) && point_4_bool(i)
                ;
            else
                disp("the following UAVs do not see the UGV");
                disp(i);
            end

        end
        %error("cannot recover ugv positions from all uavs");
    end
    
    % plot the actual image that is seen by one of the UAVs
    figure(500);
    k = 4;
    plotting_image_point(yn(k), point_1_bool(k), point_2_bool(k), point_3_bool(k), ...
        point_4_bool(k),x_result(k),y_result(k),point_1x(k),point_1y(k), ...
        point_2x(k),point_2y(k),point_3x(k),point_3y(k), point_4x(k),point_4y(k))
    
    pause(.01)

    % initialization of variable that will contain relative pose and
    % orientation of the UAVs wrt the UGV
    droneLocation = cell(form_auv.n,1);
    droneOrientation = cell(form_auv.n,1);
    % estimated_x, estimated_y represent only one element of the 
    % droneLocation variable
    estimated_x = zeros(form_auv.n,1);
    estimated_y = zeros(form_auv.n,1);


    
    for i=1:form_auv.n
        [droneLocation{i,1},droneOrientation{i,1}] = PoseReconstruction([point_1x(i) point_1y(i); ...
            point_2x(i) point_2y(i); point_3x(i) point_3y(i); point_4x(i) point_4y(i)]);
        estimated_x(i) = droneLocation{i,1}(1);
        estimated_y(i) = droneLocation{i,1}(2);
    end
    % convex_hull_estimated is a variable exploited only for plotting aims
    convex_hull_estimated = zeros(form_auv.n,2).';
    for i = 1:form_auv.n
           convex_hull_estimated(1, i) = estimated_x(i);
           convex_hull_estimated(2, i) = estimated_y(i);  
    end

end
