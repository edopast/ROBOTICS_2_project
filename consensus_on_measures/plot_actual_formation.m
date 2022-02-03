function [] = plot_actual_formation(form_auv)
%% INPUT: formatioon of UAVs variable

%% OUTPUT: no variable returned; the function has only the aims to plot the 
%  actual formation of UAVs, so as to make a comparison wrt estimated ones

    convex_hull = zeros(form_auv.n,2).';

    for i=1:form_auv.n
       convex_hull(1, i) = form_auv.xw(i,1);
       convex_hull(2, i) = form_auv.yw(i,1);
    end
    
    convex_hull = convex_hull.';
    [k,av] = convhull(convex_hull);
    plot(convex_hull(:,1),convex_hull(:,2),'.')
    hold on
    plot(convex_hull(k,1),convex_hull(k,2))

end
 