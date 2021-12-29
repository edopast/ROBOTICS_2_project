function [auv] = polar2rel(auv, ugv)
%% revert relative positions from polar positions

auv.xr = auv.rho.*cos(auv.th);
auv.yr = auv.rho.*sin(auv.th);
end