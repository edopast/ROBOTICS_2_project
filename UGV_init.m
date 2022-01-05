% Initialize the Unicycle model
run initializeUnicycleModel.m

% Generate the trajectory
run UGV_trajectory_generation.m

% Set the controller parameters
run UGV_ctrl_params.m

%% simulate
%open_system('UGV_21a.slx');
out = sim('UGV_21a.slx', 'ReturnWorkspaceOutputs', 'on');

%% Timesteps visualization

figure(4)
axis([-3 2 -0.5 3]);
title('UGV trajectory x10 speed');
grid on

for i = 1:(T/Ts-1)
    plot(pos_trajectory.signals.values(i,1), pos_trajectory.signals.values(i,2), 'ob', 'MarkerFaceColor','b');
    hold on
%     real_time = 0.001*i; %[s]
%     time_index_simulation = 1;
%     for j=1:size(out.UGVRealPose.time,1)
%        next_simulation_time = out.UGVRealPose.time(j+1,1);
%        if next_simulation_time>real_time
%            time_index_simulation = j;
%            break;
%        end
%     end
%     
%     plot(out.UGVRealPose.signals.values(time_index_simulation,1), out.UGVRealPose.signals.values(time_index_simulation,2), 'or', 'MarkerFaceColor','r');
%     
    %hold off
    %pause(0.0001)
end

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