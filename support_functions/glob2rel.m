function [auv] = glob2rel(auv, ugv)
%% revert relative positions from global positions

x = auv.xw - ugv.xw;               
y = auv.yw - ugv.yw;  
z = auv.zw; 

auv.xr = cos(ugv.hw) * x + sin(ugv.hw) * y;
auv.yr = -sin(ugv.hw) * x + cos(ugv.hw) * y;
auv.zr = z;

auv.rollr = auv.rollw ; 
auv.pitchr = auv.pitchw;
auv.yawr = auv.yaww - ugv.hw;

end