function plotFlock(auv,ugv,cam,labels)
%% Function used to plot the agents position and their orientation,plus the fov of the auvs
%% Plot auvs and ugv


plot([auv.xw],[auv.yw],'or','MarkerSize',4,'MarkerFaceColor','r')
text([auv.xw],[auv.yw],labels(1:end-1),'VerticalAlignment','top','HorizontalAlignment','center')
hold on;
plot([ugv.xw],[ugv.yw],'h','MarkerSize',10,'MarkerFaceColor','b')
text([ugv.xw],[ugv.yw],labels(end),'VerticalAlignment','top','HorizontalAlignment','center')
%% Compute lines to be plotted
xd = auv.xw+0.3*cos(auv.yaww);
yd = auv.yw+0.3*sin(auv.yaww);

ugv_xd = ugv.xw+0.3*cos(ugv.hw);
ugv_yd = ugv.yw+0.3*sin(ugv.hw);

xdl = auv.xw+0.5*cos(auv.yaww+cam.fov.y/2);
ydl = auv.yw+0.5*sin(auv.yaww+cam.fov.y/2);
xdr = auv.xw+0.5*cos(auv.yaww-cam.fov.y/2);
ydr = auv.yw+0.5*sin(auv.yaww-cam.fov.y/2);

%% Plot heading direction, and camera FoV (in x)


plot([ugv.xw ugv_xd], [ugv.yw ugv_yd],'b');

for i = 1: auv.n
  plot([auv.xw(i) xd(i)], [auv.yw(i) yd(i)],'r');
  plot([auv.xw(i) xdl(i)], [auv.yw(i) ydl(i)],'g--');
  plot([auv.xw(i) xdr(i)], [auv.yw(i) ydr(i)],'g--');
end
hold off;

end
