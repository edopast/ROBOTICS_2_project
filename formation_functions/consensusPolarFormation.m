function [agents] = consensusPolarFormation(agents, ugv, formation, progress_rate, depart_alarm_rate)
%% Function that compute the position at the next iteration of the agentss with polar consensus

% Parameters description:
% agents: AUVs data structure
% ugv: ugv data structure
% formation_r: radius of the circumference describing the formation
% new: specify if a new AUV entered the formation, and so a new formation
% has to be calculated, the store parameters are cleared
% progress_rate: progress rate for the consensus convergence

    new = formation.update;
    if(new)
        agents.dth_d = zeros(agents.n,1);
        if isfield(agents,'graph')
            agents = rmfield(agents,"graph");
        end
    end
    
    % Retrieve agents positions wrt robot frame
    % agents = glob2rel(agents, ugv);
    
    % Retrieve polar coords
    agents = rel2polar(agents, ugv);
    
         
%% Consensus on theta to formation

    % every AUV measure the theta angle difference (dth) 
    % wrt the previous AUV.
    % Consensus between all the AUV's dth is used in order to obtain the
    % average value of th_d that every AUV must keep wrt the previous AUV.
    % consensus is performed within the 2 neighboroud AUVs.
    
    % we force the theta value of the first auv to be zero
  
    temp_th = agents.th(1);
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
    agents.graph.D_deg = 2 * eye(agents.n);
    
    % for the adjacency matrix, first we define the edges:
    agents.graph.edges = [agents.id(1:agents.n-1), agents.id(2:agents.n);
                           agents.id(agents.n),     agents.id(1)         ];
         
    % define the graph
    agents.graph.G = graph(agents.graph.edges(:,1),agents.graph.edges(:,2));
    
    % finally the adjacency matrix
    agents.graph.A = full(adjacency(agents.graph.G));
    
    % the Laplacian matrix
    agents.graph.L = agents.graph.D_deg - agents.graph.A;
    
    % define the value for epsilon and the P matrix, such that P = I - eL
    eps = 1/3;
    P_dth = eye(agents.n) - eps*agents.graph.L;
    
    
    % update delta -> obtain delta "desired" with consensus
    if(new) % compute delta desired based on actual measurement
        agents.dth_d = P_dth * agents.dth;
    else % update delta desired
        progress = 2;
        while progress > progress_rate
            temp = agents.dth_d;
            agents.dth_d = P_dth * agents.dth_d;
            progress = (temp-agents.dth_d).'*(temp-agents.dth_d);
        end
    end
    
    
    
%% Theta and rho controllers

    % theta desired is defined according to the auv's ID
    if temp_th > pi
        temp_th = temp_th - 2*pi;
    end
    agents.th(1) = temp_th;
    
    % theta desired is defined according to the auv's ID
    agents.th_d = (agents.id-1).* agents.dth_d;
    agents.yawr_d = - pi + agents.th;
    
    agents.th_err = agents.th_d - agents.th;
    agents.yawr_err = agents.yawr_d - agents.yawr;
    
    for i = 1 : agents.n
        
        if agents.th_err(i) > pi
            agents.th_err(i) = - agents.th_err(i);
        end
        
        if agents.yawr_err(i) > pi
%             agents.yawr_err(i) = - agents.yawr_err(i);
        end
        
    end
    
    
    % Controller gains
    agents.K_rho = 0.08 * ones(agents.n,1);
    agents.K_th  = 0.03 * ones(agents.n,1);
    
    % Compute altitude and rho desired references
    
    agents.rho_d = zeros(agents.n,1);
    agents.zr_d  = zeros(agents.n,1);
    
    for i = 1 : agents.n
        if abs(agents.th_err(i)) > depart_alarm_rate
            agents.rho_d(i) = formation.r   + agents.id(i) * formation.offset;
            agents.zr_d(i)  = formation.alt + agents.id(i) * formation.offset;
        else
            agents.rho_d(i) = formation.r;
            agents.zr(i)    = formation.alt;
        end
    end
    
    agents.rho_err = agents.rho_d - agents.rho;
    agents.zr_err  = agents.zr_d  - agents.zr;
   

    % P-controllers
    
    variation.th  = agents.K_th  .* agents.th_err;
    variation.rho = agents.K_rho .* agents.rho_err;
    
    % Take in to account the speed saturation
    for i = i : agents.n
        if abs(variation.th(i)) > (0.2/formation.r)
            variation.th(i) = sign(variation.th(i)) * (0.2/formation.r);
        end
        
        if abs(variation.rho(i)) > 0.2
            variation.rho(i) = sign(variation.rho(i)) * 0.2;
        end
    end
    agents.th   = agents.th   +  variation.th;
    agents.rho  = agents.rho  +  variation.rho;
    agents.zr   = agents.zr   +  agents.K_rho .* agents.zr_err;
    agents.yawr = agents.yawr +  0.09 * agents.yawr_err;

%     agents.yawr = agents.yawr_d;
    
    % Update cartesian coords in the two frames and get the travelled dist
    prev_pos.xr = agents.xr;
    prev_pos.yr = agents.yr;
    
    agents = polar2rel(agents, ugv);
    agents = rel2glob(agents, ugv);
    
    d_travel_x = agents.xr - prev_pos.xr;
    d_travel_y = agents.yr - prev_pos.yr;
    agents.travel_dist = sqrt(d_travel_x.*d_travel_x + d_travel_y.*d_travel_y); 
    
    
   end