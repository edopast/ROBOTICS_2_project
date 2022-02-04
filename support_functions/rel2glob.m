function [auv] = rel2glob(auv, ugv)
%% revert global positions from relative positions

auv.xw = auv.xr*cos(ugv.hw) - auv.yr*sin(ugv.hw)+ ugv.xw;
auv.yw = auv.xr*sin(ugv.hw) + auv.yr*cos(ugv.hw)+ ugv.yw;
auv.zw = auv.zr;

auv.rollw = auv.rollr;
auv.pitchw = auv.pitchr;
auv.yaww = auv.yawr + ugv.hw;
end