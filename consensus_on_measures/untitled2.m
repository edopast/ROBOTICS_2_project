    ugv_xw = 0;   
    ugv_yw = 0; 
    ugv_zw = 0;
% the motion of the UGV is simulated at UAVs level (offset variable)
    x_offset = ugv.trajectory.signals.values(2000,1);
    y_offset = ugv.trajectory.signals.values(2000,2);
    
    form_ref_x= zeros(form_auv.n,1);
    form_ref_y= zeros(form_auv.n,1);
    form_ref_z= zeros(form_auv.n,1);
   
    for i = 1:form_auv.n
        form_ref_x(i,1) = form_auv.xw(i,1)-x_offset;
        form_ref_y(i,1) = form_auv.yw(i,1)-y_offset;
        form_ref_z(i,1) = form_auv.zw(i,1);
        % da modificare l'altezza ottenuta dalla formazione 
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
    
    P1 = [ugv.P1(1); ugv.P1(2); ugv.P1(3)]; %[m]
    P2 = [ugv.P2(1); ugv.P2(2); ugv.P2(3)]; %[m]
    P3 = [ugv.P3(1); ugv.P3(2); ugv.P3(3)]; %[m]
    P4 = [ugv.P4(1); ugv.P4(2); ugv.P4(3)]; %[m]
    
    for i = 1:form_auv.n
        
        [yn(i),x_result(i),y_result(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollw(i,1) form_auv.pitchw(i,1) form_auv.yaww(i,1)], ...
            [ugv_xw ugv_yw 0]);
    
        [point_1_bool(i),point_1x(i),point_1y(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollw(i,1) form_auv.pitchw(i,1) form_auv.yaww(i,1)], P1.');
    
        [point_2_bool(i),point_2x(i),point_2y(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollw(i,1) form_auv.pitchw(i,1) form_auv.yaww(i,1)], P2.');
    
        [point_3_bool(i),point_3x(i),point_3y(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollw(i,1) form_auv.pitchw(i,1) form_auv.yaww(i,1)], P3.');
    
        [point_4_bool(i),point_4x(i),point_4y(i)] = computeCameraImage([form_ref_x(i,1) form_ref_y(i,1) ...
            form_ref_z(i,1)], [form_auv.rollw(i,1) form_auv.pitchw(i,1) form_auv.yaww(i,1)], P4.');
    
    end
    
    % bool variable to check if all UAVs had the capability of see the UGV
    UGV_seen = 0;
    for i = 1:form_auv.n
        if yn(i) && point_1_bool(i) && point_2_bool(i) && point_3_bool(i) && point_4_bool(i)
            UGV_seen = UGV_seen+1;
        end
    end

    if(UGV_seen == form_auv.n)
        ;
    else 
        disp("cannot recover ugv positions from all uavs");
    end
    % plot the actual image that is seen by one of the UAVs
    k = 3;
    plotting_image_point(yn(k), point_1_bool(k), point_2_bool(k), point_3_bool(k), ...
        point_4_bool(k), x_result(k),y_result(k), point_1x(k),point_1y(k), ...
        point_2x(k),point_2y(k),point_3x(k),point_3y(k), point_4x(k),point_4y(k))
    hold on 
    [estimated_after.x, estimated_after.y,... 
        convex_hull_estimated_after, points_image_plane_y] = pose_estimation(ugv,form_auv,2000);

    

    centre_y = yaw_error(points_image_plane_y, form_auv);
% disp(centre_x(1));
    plot(centre_y(3), 0, 'o');
    hold on 
    waitforbuttonpress;
    




