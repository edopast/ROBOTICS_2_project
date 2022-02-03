function [] = plot_with_convexhull(convex_hull)
%% INPUT: convexhull variable

%% OUTPUT: no variable returned; the function has only the aims to plot the 
%  convexhull that is given as input 

    convex_hull_estimated = convex_hull.';
    [k,av] = convhull(convex_hull_estimated);
    plot(convex_hull_estimated(:,1),convex_hull_estimated(:,2),'.')
    hold on
    plot(convex_hull_estimated(k,1),convex_hull_estimated(k,2))
end