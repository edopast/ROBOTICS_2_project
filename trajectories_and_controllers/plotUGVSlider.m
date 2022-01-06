function plotUGVSlider(src,event,pos_trajectory,out,legendInfo)

    i = 10*(round(src.Value))+1;
    src.Parent;
    plot(pos_trajectory.signals.values(i,1), pos_trajectory.signals.values(i,2), 'ob','MarkerSize', 5, 'MarkerFaceColor','b');
    axis([-3 1.5 -1 3]);
    time = 0.001*i; %[s]
    title('UGV position time = {}',time);
    grid on
    hold on

    time_index_simulation = round(i*size(out.UGVRealPose.signals.values,1)/size(pos_trajectory.signals.values,1));
    
    plot(out.UGVRealPose.signals.values(time_index_simulation,1), out.UGVRealPose.signals.values(time_index_simulation,2), 'pr','MarkerSize', 6.5, 'MarkerFaceColor','r');
    
    legendInfo(1, 1) = sprintf('Desired');
    legendInfo(2, 1) = sprintf('True');

    legend(legendInfo(:,:),'Location','SouthEast');
    xlabel('x --> [m]')
    ylabel('y --> [m]')
    
    hold off
end

