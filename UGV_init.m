% Initialize the Unicycle model
run initializeUnicycleModel.m

% Generate the trajectory
run UGV_trajectory_generation.m

% Set the controller parameters
run UGV_ctrl_params.m

%% simulate
open_system('UGV_21a.slx');
out = sim('UGV_21a.slx', 'ReturnWorkspaceOutputs', 'on');

%% Timesteps visualization


f=figure(1);
axis([-3 2 -0.5 3]);
title('UGV trajectory');
grid on

c = uicontrol(f,'Style','slider','Position',[50 5 450 15], 'Min', 0, 'Max', T/Ts-2,'SliderStep', [0.0005 0.005], 'BackgroundColor', '#FFFFFF');
c.String = 'Time [s]';
c.Callback = {@plotUGVSlider,pos_trajectory,out};

%% plot desired trajectory
hold off
%plot(head_trajectory.time, head_trajectory.signals.values);
figure(2)
hold on
plot(pos_trajectory.signals.values(:,1), pos_trajectory.signals.values(:,2), 'b');
axis([-3 2 -0.5 3]);
grid on;
title('UGVtrajectory');
legendInfo=string(zeros(2,1));
legendInfo(1, 1) = sprintf('Desired');

% plot real trajectory
plot(out.UGVRealPose.signals.values(:,1), out.UGVRealPose.signals.values(:,2), 'r');
legendInfo(2, 1) = sprintf('True');

legend(legendInfo(:,:),'Location','SouthEast');
xlabel('x --> [m]')
ylabel('y --> [m]')
