function [form_auv] = addToFormation(form_auv,auv,index)
%% copy auv data in index to the formation struct
form_auv.n = form_auv.n + 1;

form_auv.id(form_auv.n) = auv.id(index);           
form_auv.xw(form_auv.n) = auv.xw(index);
form_auv.yw(form_auv.n) =  auv.yw(index);   
form_auv.zw(form_auv.n) =  auv.zw(index);   

form_auv.rollw(form_auv.n)  =  auv.rollw(index);             
form_auv.pitchw(form_auv.n) =  auv.pitchw(index);        
form_auv.yaww(form_auv.n)   =  auv.yaww(index);         

form_auv.xr(form_auv.n) = auv.xr(index);                 
form_auv.yr(form_auv.n) = auv.yr(index);     
form_auv.zr(form_auv.n) = auv.zr(index);   

form_auv.rollr(form_auv.n)  = auv.rollr(index);   
form_auv.pitchr(form_auv.n) = auv.pitchr(index);   
form_auv.yawr(form_auv.n)   = auv.yawr(index);   

end
