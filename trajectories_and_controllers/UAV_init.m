clear all
close all
% Initialize the Unicycle model
run initializeQuadrotorModel.m

% Generate the trajectory
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
%hold on
plot3(p_ref.signals.values(:,1), p_ref.signals.values(:,2), p_ref.signals.values(:,3), 'b', 'LineWidth', 1);
grid on;
title('UAVtrajectory');
legendInfo=string(zeros(2,1));
legendInfo(1, 1) = sprintf('Desired');

% plot real trajectory
plot(AGVRealPose.Pose.signals.values(:,1), AGVRealPose.Pose.signals.values(:,2), 'r-.', 'LineWidth', 1);
legendInfo(2, 1) = sprintf('True');

legend(legendInfo(:,:),'Location','SouthEast');
xlabel('x --> [m]')
ylabel('y --> [m]')