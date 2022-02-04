function [struct] = removeAuvFrom(struct,index)
%% remove a list of auvs from the struct

for i = flip(index)
    struct.n = struct.n - 1;
    
    struct.id(i) = [];           
    struct.xw(i) = [];      
    struct.yw(i) = [];        
    struct.zw(i) = [];      

    struct.rollw(i)  =  [];                
    struct.pitchw(i) =  [];      
    struct.yaww(i)   =  [];            

    struct.xr(i) = [];                 
    struct.yr(i) = [];         
    struct.zr(i) = [];      

    struct.rollr(i)  = [];       
    struct.pitchr(i) = [];       
    struct.yawr(i)   = [];  
end
end
