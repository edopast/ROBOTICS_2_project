function [agents] = consensusPolarFormation(agents, ugv, f_radius, new)
%% Function that compute the position at the next iteration of the agentss with polar consensus

% Paramiters description:
% agents: AUVs data structure
% ugv: ugv data structure
% f_radius: radius of the circumference describing the formation
% new: specify if a new AUV entered the formation, and so a new formation
% has to be calculated, the store parameters are cleared
    
    if(new)
        agents.th_dd = zeros(agents.n,1);
    end
    
    % Retrieve agents positions wrt robot frame
    agents = glob2rel(agents, ugv);
    
    % Retrieve polar coords
    agents = rel2polar(agents, ugv);
    
%% Rho distance controller

    % Gain for the rho position controller
    K.rho = 0.1;
    
    % Normalized distance error
    agents.rho_err = agents.rho - ones(agents.n , 1) * f_radius;
    agents.rho_err_norm = agents.rho_err./agents.rho;
    
    % Control matrix for rho
    P_rho = [eye(agents.n)-diag(K.rho*agents.rho_err)   K.rho*agents.rho_err;
             zeros(1,agents.n)                     1               ];
         
%% Consensus on theta to formation

    % every AUV measure the theta angle difference (th_d) 
    % wrt the previous AUV.
    % Consensus between all the AUV's th_d is used in order to obtain the
    % average value of th_d that every AUV must keep wrt the previous AUV.
    % consensus is performed within the 2 neighboroud AUVs.
    
    agents.th_d =[   agents.th(1)+2*pi - agents.th(agents.n);
                    agents.th(2:agents.n) - agents.th(1:agents.n-1)];

    % th_d structure: (v1' = theta(v1) +2pi)
    % | v1'-vn|    | th_d1 |
    % | v2-v1 |    | th_d2 |
    % | v3-v2 |  = | th_d3 |
    % | ..... |    | ..... |
    % | ..... |    | ..... |
    % |vn-vn-1|    | th_dn |
    
    % --------------------- DA CAMBIARE USANDO IL GRAFO
            % definitio of the average consensus matrix:
            coeff_th_d = 1/5 * ones(1, agents.n);

            % matrice di consenso per delta theta:
            % se stesso
            P_th_d =  diag(coeff_th_d);
            % drone prima
            P_th_d =  P_th_d + diag(coeff_th_d(2:agents.n),-1)  + diag(coeff_th_d(1),agents.n-1); 
            % drone dopo       
            P_th_d =  P_th_d + diag(coeff_th_d(2:agents.n),+1)  + diag(coeff_th_d(1),-(agents.n-1));
            % due droni prima
            P_th_d =  P_th_d + diag(coeff_th_d(3:agents.n),-2)  + diag(coeff_th_d(1:2),agents.n-2); 
            % due droni dopo       
            P_th_d =  P_th_d + diag(coeff_th_d(3:agents.n),+2)  + diag(coeff_th_d(1:2),-(agents.n-2));
    
    
    % update delta -> obtain delta "desired" with consensus
    if(new) % compute delta desired based on actual measurement
        agents.th_dd = P_th_d * agents.th_d;
    else % update delta desired
        agents.th_dd = P_th_d * agents.th_dd;
    end
    
    
    % normalized theta error (actual delta theta - desired delta theta)
    agents.th_err = 0.5*(agents.th_d - agents.th_dd) / (2*pi);
    
    
    %agents.th_d_f =[    agents.th(1:agents.n-1) - agents.th(2:agents.n);
    %                 agents.th(agents.n) - agents.th(1)-2*pi     ];
   

    % Control matrix for theta position:
    % the first AUV is simply sent to theta = 0
    % other AUVs are moved according to the error between the previous and
    % the the next neighbors.
  
     
    % | 0.8  0   0   0  . . . .  0.2 |   | 0.8  0   0   0  . . .  | 0.2 |
    % |  *   *   0   0  . . . .   0  |   |                        |  0  |
    % |  0   *   *   0  . . . .   0  |   |                        |  0  |
    % |  0   0   *   *  . . . .   0  | = |        look_around       |  0  |
    % |  . . . .. . . . . .*  *   0  |   |                        |  0  |
    % |  0 . . .. . . . . . . 0   1  |   |  0 . . .. . . . . .  0 |  1  |
    
    look_around = eye(agents.n) -2*diag(agents.th_err) + diag(agents.th_err(2:agents.n),-1) + diag(agents.th_err(2:agents.n),1) ;
    look_around = look_around + diag(agents.th_err(1),-(agents.n-1));
    
    
    P_th = [ 0.8     zeros(1, agents.n-1)      0.2;
             look_around(2:agents.n,:)           zeros(agents.n-1,1);
             zeros(1,agents.n)                 1               ];
    
    
    % System update
    state.rho = [agents.rho; 0];
    state.th  = [agents.th;  0];
  
    state.rho = P_rho * state.rho;
    state.th  = P_th  * state.th;
    
    agents.rho = state.rho(1:agents.n);
    agents.th  = state.th(1:agents.n);
    
    % Update cartesian coords in the two frames and get the travelled dist
    prev_pos.xr = agents.xr;
    prev_pos.yr = agents.yr;
    agents = polar2rel(agents, ugv);
    agents = rel2glob(agents, ugv);
    d_travel_x = agents.xr - prev_pos.xr;
    d_travel_y = agents.yr - prev_pos.yr;
    agents.travel_dist = sqrt(d_travel_x.*d_travel_x + d_travel_y.*d_travel_y); 

    
    
   end