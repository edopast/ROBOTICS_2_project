function [form_auv] = addToFormation(form_auv,auv,index)
%% copy auv data in index to the formation struct
form_auv.n = form_auv.n + 1;

form_auv.id(form_auv.n,1) = auv.id(index);           
form_auv.xw(form_auv.n,1) = auv.xw(index);
form_auv.yw(form_auv.n,1) =  auv.yw(index);   
form_auv.zw(form_auv.n,1) =  auv.zw(index);   

form_auv.rollw(form_auv.n,1)  =  auv.rollw(index);             
form_auv.pitchw(form_auv.n,1) =  auv.pitchw(index);        
form_auv.yaww(form_auv.n,1)   =  auv.yaww(index);         

form_auv.xr(form_auv.n,1) = auv.xr(index);                 
form_auv.yr(form_auv.n,1) = auv.yr(index);     
form_auv.zr(form_auv.n,1) = auv.zr(index);   

form_auv.rollr(form_auv.n,1)  = auv.rollr(index);   
form_auv.pitchr(form_auv.n,1) = auv.pitchr(index);   
form_auv.yawr(form_auv.n,1)   = auv.yawr(index);   

% form_auv.record.traj = auv.record.traj;

end
