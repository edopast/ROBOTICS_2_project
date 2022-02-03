function [] = correct_formation(ugv, form_auv, t)
%% INPUT:  - ugv: UGV structure variable (ugv); 
%          - form_auv: formation structure variable (form_auv);
%          - t: time instant in whic evaluate the UGV trajectory;
        

%% OUTPUT: no variable returned; the function has only the aims to plot the 
%  ideal formation of UAVs, so as to make a comparison wrt estimated ones
%  (and also the actual ones)

n=form_auv.n;    % number of vertex of the regular polygon formation 
r=2;        % ray associated to the formation

ugv_xw = ugv.trajectory.signals.values(t,1);  
ugv_yw = ugv.trajectory.signals.values(t,2);

x = ugv_xw;
y = ugv_yw;

%% plotting stuff...
convex_hull = zeros(n,2).';
point = zeros(2,1);

% using this formulation, i am assuming that there is an agent in
% coordinate (rho,0); all the other agents are disposed starting from this
% coordinate

for i = 0:n
       point(1) = x + r * cos(2 * pi * i / n);    
       point(2) = y + r * sin(2 * pi * i / n); 
       point = point.';
       convex_hull(1, i+1) = point(1);
       convex_hull(2, i+1) = point(2);  
end

% circumbscribed circle
ray = r;
xc = x;
yc = y;
theta = linspace(0,2*pi);
x = ray*cos(theta) + xc;
y = ray*sin(theta) + yc;

% convexhull definition given the vertix of the geometric pattern of the
% UAV formation
convex_hull = convex_hull.';
[k,av] = convhull(convex_hull);
plot(x,y,'LineWidth',1);
hold on
plot(convex_hull(:,1),convex_hull(:,2),'.')
hold on
plot(ugv_xw,ugv_yw, '.')
plot(convex_hull(k,1),convex_hull(k,2), 'LineWidth',1)


end
