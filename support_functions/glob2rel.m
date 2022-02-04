function [auv] = glob2rel(auv, ugv)
%% revert relative positions from global positions
auv.xr = auv.xw - ugv.xw;               
auv.yr = auv.yw - ugv.yw;  
auv.zr = auv.zw; 

auv.rollr = auv.rollw ; 
auv.pitchr = auv.pitchw;
auv.yawr = auv.yaww - ugv.hw;

end