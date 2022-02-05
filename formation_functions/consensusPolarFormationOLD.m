function [agents] = consensusPolarFormation(agents, ugv, formation_r, new, n)
%% Function that compute the position at the next iteration of the agentss with polar consensus

% Paramiters description:
% agents: AUVs data structure
% ugv: ugv data structure
% formation_r: radius of the circumference describing the formation
% new: specify if a new AUV entered the formation, and so a new formation
% has to be calculated, the store parameters are cleared
% n: number of consensus iteration 
    
    if(new)
        agents.dth_d = zeros(agents.n,1);
    end
    
    % Retrieve agents positions wrt robot frame
    agents = glob2rel(agents, ugv);
    
    % Retrieve polar coords
    agents = rel2polar(agents, ugv);
    
%% Rho distance controller

    % Gain for the rho position controller
    K.rho = 0.1;
    
    % Normalized distance error
    agents.rho_err = agents.rho - ones(agents.n , 1) * formation_r;
    agents.rho_err_norm = agents.rho_err./agents.rho;
    
    % Control matrix for rho
    P_rho = [eye(agents.n)-diag(K.rho*agents.rho_err)   K.rho*agents.rho_err;
             zeros(1,agents.n)                     1               ];
         
%% Consensus on theta to formation

    % every AUV measure the theta angle difference (dth) 
    % wrt the previous AUV.
    % Consensus between all the AUV's dth is used in order to obtain the
    % average value of th_d that every AUV must keep wrt the previous AUV.
    % consensus is performed within the 2 neighboroud AUVs.
    
    % we force the theta value of the first auv to be zero
  
    agents.th(1) = 0;
    
    agents.dth =[   agents.th(1)+2*pi - agents.th(agents.n);
                    agents.th(2:agents.n) - agents.th(1:agents.n-1)];
    

    % dth structure: (v1' = theta(v1) +2pi)
    % | v1'-vn|    | th_d1 |
    % | v2-v1 |    | th_d2 |
    % | v3-v2 |  = | th_d3 |
    % | ..... |    | ..... |
    % | ..... |    | ..... |
    % |vn-vn-1|    | th_dn |
    
    % construction of the average consensus for dth using Laplacian-based design:
    % L = Ddegree - Adjacency
    
    % every node of the graph has 2 degree
    D_deg = 2 * eye(agents.n);
    
    % for the adjacency matrix, first we define the edges:
    edges = [agents.id(1:agents.n-1), agents.id(2:agents.n);
             agents.id(agents.n),     agents.id(1)         ];
         
    % define the graph
    G = graph(edges(:,1),edges(:,2));
    
    % finally the adjacency matrix
    A = full(adjacency(G));
    
    % the Laplacian matrix
    L = D_deg - A;
    
    % define the value for epsilon and the P matrix, such that P = I - eL
    eps = 1/3;
    P_dth = eye(agents.n) - eps*L;
    
    
    disp(agents.n)
    i = 1;
    % update delta -> obtain delta "desired" with consensus
    if(new) % compute delta desired based on actual measurement
        agents.dth_d = P_dth * agents.dth;
    else % update delta desired
        progress = 2;
        while progress > 0.1
            temp = agents.dth_d;
            agents.dth_d = P_dth * agents.dth_d;
            progress = (temp-agents.dth_d).'*(temp-agents.dth_d);
        end
    end

    % normalized delta theta error (actual delta theta - desired delta theta)
    agents.dth_err = 0.5*(agents.dth - agents.dth_d) / (2*pi);
   
    % Control matrix for theta position:
    % the first AUV is simply sent to theta = 0
    % other AUVs are moved according to the error between the previous and
    % the the next neighbors.
  
     
    % | 0.8  0   0   0  . . . .  0.2 |   | 0.8  0   0   0  . . .  | 0.2 |
    % |  *   *   0   0  . . . .   0  |   |                        |  0  |
    % |  0   *   *   0  . . . .   0  |   |                        |  0  |
    % |  0   0   *   *  . . . .   0  | = |        look_around     |  0  |
    % |  . . . .. . . . . .*  *   0  |   |                        |  0  |
    % |  0 . . .. . . . . . . 0   1  |   |  0 . . .. . . . . .  0 |  1  |
    
    look_around = eye(agents.n) -2*diag(agents.dth_err) + diag(agents.dth_err(2:agents.n),-1) + diag(agents.dth_err(2:agents.n),1) ;
    look_around = look_around + diag(agents.dth_err(1),-(agents.n-1));
    
    
    P_th = [ 0.5     zeros(1, agents.n-1)      0.5;
             look_around(2:agents.n,:)         zeros(agents.n-1,1);
             zeros(1,agents.n)                 1               ];
    

    
    % System update
    state.rho = [agents.rho; 0];
    %state.th  = [agents.th;  0];
  
    state.rho = P_rho * state.rho;
    %state.th  = P_th  * state.th;
    
    agents.rho = state.rho(1:agents.n);
   % agents.th  = state.th(1:agents.n);
    
    agents.th = 0.1*(agents.id-1).* agents.dth_d + 0.9 * agents.th;
    
    % Update cartesian coords in the two frames and get the travelled dist
    prev_pos.xr = agents.xr;
    prev_pos.yr = agents.yr;
    
    agents.yawr = pi + agents.th;
    
    
    
    agents = polar2rel(agents, ugv);
    agents = rel2glob(agents, ugv);
    d_travel_x = agents.xr - prev_pos.xr;
    d_travel_y = agents.yr - prev_pos.yr;
    agents.travel_dist = sqrt(d_travel_x.*d_travel_x + d_travel_y.*d_travel_y); 
    
   
    
    
   end