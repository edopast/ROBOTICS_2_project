function [auv] = rel2glob(auv, ugv)
%% revert global positions from relative positions

auv.xw = auv.xr + ugv.xw;
auv.yw = auv.yr + ugv.yw;
auv.zw = auv.zr;

auv.rollw = auv.rollr;
auv.pitchw = auv.pitchr;
auv.yaww = auv.yawr + ugv.hw;
end