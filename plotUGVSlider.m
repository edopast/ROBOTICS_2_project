function plotUGVSlider(src,event,pos_trajectory,out)
        i = round(src.Value)+1;
        src.Parent;
        plot(pos_trajectory.signals.values(i,1), pos_trajectory.signals.values(i,2), 'ob', 'MarkerFaceColor','b');
        axis([-3 2 -0.5 3]);
        title('UGV trajectory');
        grid on
        hold on
        
    real_time = 0.001*i %[s]
    time_index_simulation = 1;
    for j=1:size(out.UGVRealPose.time,1)
       next_simulation_time = out.UGVRealPose.time(j+1,1);
       if next_simulation_time>real_time
           time_index_simulation = j;
           break;
       end
    end
    
    plot(out.UGVRealPose.signals.values(time_index_simulation,1), out.UGVRealPose.signals.values(time_index_simulation,2), 'or', 'MarkerFaceColor','r');
    hold off
end

