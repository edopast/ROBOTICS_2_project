function plotting_image_point(bool_result, bool_point1, bool_point2, bool_point3, bool_point4, ...
    result_x, result_y, point1x, point1y, point2x, point2y, point3x, point3y,point4x, point4y)
%% INPUT: set of points recovered by computeCameraImage function; 
%  the point are in the order [bool_ugv, bool_p1, ..., bool_p4, x_ugv, y_ugv, x_p1,y_p1,  .., x_p4, y_p4]

%% OUTPUT: the function has no returned variable; it aims is to show what UAVs are actually
%  seen through their camera (after point computation by computeCameraImage function)

    if bool_result && bool_point1 && bool_point2 && bool_point3 && bool_point4
        plot(result_y(1),result_x(1),'or','MarkerSize',3,'MarkerFaceColor','r')
        hold on 
        plot(point1y(1),point1x(1),'or','MarkerSize',3,'MarkerFaceColor','b')
        hold on 
        plot(point2y(1),point2x(1),'or','MarkerSize',3,'MarkerFaceColor','b')
        hold on 
        plot(point3y(1),point3x(1),'or','MarkerSize',3,'MarkerFaceColor','b')
        hold on 
        plot(point4y(1),point4x(1),'or','MarkerSize',3,'MarkerFaceColor','b')
        hold on 
        axis([-3.16*10^-3 3.16*10^-3  -2.37*10^-3  2.37*10^-3])
        title('image plane vision UAV')
    else
        % if one of the UAVs is not able to see the UGV, then it is plotted
        % a string alert
        disp("UGV is not seen by the image plane")
    end
end
