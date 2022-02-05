function [centre_x, centre_y] = formation_centre(form_auv)
    n = form_auv.n;
    sum_x = 0;
    sum_y = 0;
    
    for i=1:n

        sum_x =  sum_x + form_auv.xw(i);
        sum_y =  sum_y + form_auv.yw(i);

    end

    centre_x = sum_x/n;
    centre_y = sum_y/n;

end