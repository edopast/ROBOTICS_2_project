clear all
close all

% Generate the trajectory
run UGV_init.m
close all

% Initialize the Unicycle model
run initializeQuadrotorModel.m

run UAV_trajectory_generation.m

% Set the controller parameters
run UAV_ctrl_params.m

%% simulate
open_system('UAV_2021_a.slx');
UAVRealPose = sim('UAV_2021_a.slx', 'ReturnWorkspaceOutputs', 'on');
%UAVRealPose.Pose.signals.values contains 3 columns: [x y z roll pitch yaw]

%% plot desired trajectory
%hold off
figure(2)
plot3(p_ref.signals.values(:,1), p_ref.signals.values(:,2), p_ref.signals.values(:,3), 'b', 'LineWidth', 1);
hold on
grid on;
title('UAVtrajectory');
legendInfo=string(zeros(2,1));
legendInfo(1, 1) = sprintf('Desired');
legendInfo(2, 1) = sprintf('True');

% plot real trajectory
plot3(UAVRealPose.Pose.signals.values(:,1), UAVRealPose.Pose.signals.values(:,2), UAVRealPose.Pose.signals.values(:,3), 'r-.', 'LineWidth', 1);

% and also yaw angle (occasionally)
for j=1:20:100
    UAV_xd = UAVRealPose.Pose.signals.values(j,1) + 0.3*cos(UAVRealPose.Pose.signals.values(j,6));
    UAV_yd = UAVRealPose.Pose.signals.values(j,2) + 0.3*sin(UAVRealPose.Pose.signals.values(j,6));
    plot3([UAVRealPose.Pose.signals.values(j,1) UAV_xd], [UAVRealPose.Pose.signals.values(j,2) UAV_yd], [UAVRealPose.Pose.signals.values(j,3) UAVRealPose.Pose.signals.values(j,3)], 'g', 'LineWidth', 1.5);
end

for j=101:100:401
    UAV_xd = UAVRealPose.Pose.signals.values(j,1) + 0.3*cos(UAVRealPose.Pose.signals.values(j,6));
    UAV_yd = UAVRealPose.Pose.signals.values(j,2) + 0.3*sin(UAVRealPose.Pose.signals.values(j,6));
    plot3([UAVRealPose.Pose.signals.values(j,1) UAV_xd], [UAVRealPose.Pose.signals.values(j,2) UAV_yd], [UAVRealPose.Pose.signals.values(j,3) UAVRealPose.Pose.signals.values(j,3)], 'g', 'LineWidth', 1.5);
end

for j=401:200:(size(UAVRealPose.Pose.signals.values,1))
    UAV_xd = UAVRealPose.Pose.signals.values(j,1) + 0.3*cos(UAVRealPose.Pose.signals.values(j,6));
    UAV_yd = UAVRealPose.Pose.signals.values(j,2) + 0.3*sin(UAVRealPose.Pose.signals.values(j,6));
    plot3([UAVRealPose.Pose.signals.values(j,1) UAV_xd], [UAVRealPose.Pose.signals.values(j,2) UAV_yd], [UAVRealPose.Pose.signals.values(j,3) UAVRealPose.Pose.signals.values(j,3)], 'g', 'LineWidth', 1.5);
end

legend(legendInfo(1:2,:),'Location','SouthEast');

xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

%% Plot some graphs
hold off
figure(3)
title('X,Y,Z coordinates of the UAV')

subplot(3,1,1);
plot(p_ref.time(:,1), p_ref.signals.values(:,1), 'b')
hold on
plot(UAVRealPose.Pose.time(:,1), UAVRealPose.Pose.signals.values(:,1), 'r-.')
xlabel('time [s]')
ylabel('x [m]')
hold off

subplot(3,1,2);
plot(p_ref.time(:,1), p_ref.signals.values(:,2), 'b')
hold on
plot(UAVRealPose.Pose.time(:,1), UAVRealPose.Pose.signals.values(:,2), 'r-.')
xlabel('time [s]')
ylabel('y [m]')
hold off

subplot(3,1,3);
plot(p_ref.time(:,1), p_ref.signals.values(:,3), 'b')
hold on
plot(UAVRealPose.Pose.time(:,1), UAVRealPose.Pose.signals.values(:,3), 'r-.')
xlabel('time [s]')
ylabel('z [m]')
hold off

legend(legendInfo(1:2,:),'Location','SouthEast');

%-------

hold off
figure(4)
title('roll,pitch and yaw of the UAV')

subplot(3,1,1);
plot(UAVRealPose.Pose.time(:,1), UAVRealPose.Pose.signals.values(:,4), 'g')
xlabel('time [s]')
ylabel('roll [rad]')

subplot(3,1,2);
plot(UAVRealPose.Pose.time(:,1), UAVRealPose.Pose.signals.values(:,5), 'g')
xlabel('time [s]')
ylabel('pitch [rad]')

subplot(3,1,3);
plot(yaw_ref.time(:,1), yaw_ref.signals.values(:,1), 'b')
hold on
plot(UAVRealPose.Pose.time(:,1), UAVRealPose.Pose.signals.values(:,6), 'g-.')
xlabel('time [s]')
ylabel('yaw [rad]')
hold off

legend(legendInfo(1:2,:),'Location','SouthEast');
