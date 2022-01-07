function plotUGVSlider(src,event,ugv,UGVRealPose,legendInfo)

    i = 10*(round(src.Value))+1;
    src.Parent;
    % plot reference point
    plot(ugv.trajectory.signals.values(i,1), ugv.trajectory.signals.values(i,2), 'ob','MarkerSize', 6.5, 'MarkerFaceColor','b');
    hold on

    %% plot desired trajectory to track
    
    plot(ugv.trajectory.signals.values(:,1), ugv.trajectory.signals.values(:,2), 'k');
    
    %axis([-3 1.5 -1 3]);
    time = 0.001*i; %[s]
    title('UGV position time = {}',time);
    grid on
    
    %% plot true position
    
    time_index_simulation = round(i*size(UGVRealPose.Pose.signals.values,1)/size(ugv.trajectory.signals.values,1));
    
    plot(UGVRealPose.Pose.signals.values(time_index_simulation,1), UGVRealPose.Pose.signals.values(time_index_simulation,2), 'pr','MarkerSize', 8, 'MarkerFaceColor','r');
    
    %% Compute direction of UGV (true)
    xd = UGVRealPose.Pose.signals.values(time_index_simulation,1) + 0.15*cos(UGVRealPose.Pose.signals.values(time_index_simulation,3));
    yd = UGVRealPose.Pose.signals.values(time_index_simulation,2) + 0.15*sin(UGVRealPose.Pose.signals.values(time_index_simulation,3));
    plot([UGVRealPose.Pose.signals.values(time_index_simulation,1) xd], [UGVRealPose.Pose.signals.values(time_index_simulation,2) yd],'r', 'LineWidth',1.5);
    
    legendInfo(1, 1) = sprintf('Desired');
    legendInfo(2, 1) = sprintf('Desired Traj');
    legendInfo(3, 1) = sprintf('True');

    legend(legendInfo(:,:),'Location','SouthEast');
    xlabel('x --> [m]')
    ylabel('y --> [m]')
    
    hold off
end

