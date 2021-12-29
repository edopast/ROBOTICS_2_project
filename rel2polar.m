function [auv] = rel2polar(auv, ugv)
%% revert polar positions from relative positions

auv.rho = sqrt(auv.xr.*auv.xr + auv.yr.*auv.yr);
auv.th  = mod(atan2(auv.yr,auv.xr),2*pi);
end