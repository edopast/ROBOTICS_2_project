function [centre_x] = yaw_error(points_image_plane_y, form_auv)
    n = 4;
    sum_x = zeros(form_auv.n,1);
    centre_x = zeros(form_auv.n,1);

    for i=1:form_auv.n
        for j = 1:4
            sum_x(i) =  sum_x(i) + points_image_plane_y{i}(j);
            
        end
        
    end
    
    for i=1:form_auv.n
        sum_x(i)= sum_x(i)/n;
        centre_x(i) = sum_x(i);
    end



end