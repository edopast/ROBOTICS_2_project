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
for j=1:400:(size(UAVRealPose.Pose.signals.values,1))
    UAV_xd = UAVRealPose.Pose.signals.values(j,1) + 0.3*cos(UAVRealPose.Pose.signals.values(j,6));
    UAV_yd = UAVRealPose.Pose.signals.values(j,2) + 0.3*sin(UAVRealPose.Pose.signals.values(j,6));
    plot3([UAVRealPose.Pose.signals.values(j,1) UAV_xd], [UAVRealPose.Pose.signals.values(j,2) UAV_yd], [UAVRealPose.Pose.signals.values(j,3) UAVRealPose.Pose.signals.values(j,3)], 'g', 'LineWidth', 1.5);
end

legend(legendInfo(1:2,:),'Location','SouthEast');

xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
