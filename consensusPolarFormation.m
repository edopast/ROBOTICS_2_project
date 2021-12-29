function [agent] = consensusPolarFormation(agent, ugv, f_radius, new)
%% function that compute the position at the next iteration of the agents with polar consensus

    if(new)
        agent.th_dd = zeros(agent.n,1);
    end
    K.rho = 0.1;
    
    % Retrieve agents positions wrt robot frame
    agent = glob2rel(agent, ugv);
    
    % Retrieve polar coords
    agent = rel2polar(agent, ugv);
    
    % Normalized distance error
    agent.d_err = agent.rho - ones(agent.n , 1) * f_radius;
    agent.d_err_norm = agent.d_err./agent.rho;
    
    % Control matrix for rho
    P_rho = [eye(agent.n)-diag(K.rho*agent.d_err)   K.rho*agent.d_err;
             zeros(1,agent.n)                     1               ];
    
    % Consensus matrix per theta :
    % calcolare differenza delta (th_d) tra il drone e quello precedente;
    % fare average consensus su questo delta, in modo da trovare la media
    % dei delta che ogni drone deve avere rispetto al suo precedente 
    % e sucessivo per essere equamente distribuiti
    
    % In questo consenso consderiamo solo i droni, l'UGV non serve
    
    agent.th_d =[   agent.th(1)+2*pi - agent.th(agent.n);
                    agent.th(2:agent.n) - agent.th(1:agent.n-1)];

    % il vettore è costruito così: (v1' = theta(v1) +2pi)
    % | v1'-vn|    | th_d1 |
    % | v2-v1 |    | th_d2 |
    % | v3-v2 |  = | th_d3 |
    % | ..... |    | ..... |
    % | ..... |    | ..... |
    % |vn-vn-1|    | th_dn |
    
    % Definisco la matrice di average consensus:
    coeff_th_d = 1/5 * ones(1, agent.n);
    
    % matrice di consenso per delta theta:
    % se stesso
    P_th_d =  diag(coeff_th_d);
    % drone prima
    P_th_d =  P_th_d + diag(coeff_th_d(2:agent.n),-1)  + diag(coeff_th_d(1),agent.n-1); 
    % drone dopo       
    P_th_d =  P_th_d + diag(coeff_th_d(2:agent.n),+1)  + diag(coeff_th_d(1),-(agent.n-1));
    % due droni prima
    P_th_d =  P_th_d + diag(coeff_th_d(3:agent.n),-2)  + diag(coeff_th_d(1:2),agent.n-2); 
    % due droni dopo       
    P_th_d =  P_th_d + diag(coeff_th_d(3:agent.n),+2)  + diag(coeff_th_d(1:2),-(agent.n-2));
    
    % update delta -> obtain delta "desired" with consensus
    if(new) % calcola th_dd sulla base delle misurazioni attuali
        agent.th_dd = P_th_d * agent.th_d;
    else % calcola th_dd sulla base dei precedenti th_dd
        agent.th_dd = P_th_d * agent.th_dd;
    end
    
    
    % Ora si calcola l'errore di theta definito come:
    % delta_attuale - delta_desired -> poi normalizzato
    agent.th_err = 0.5*(agent.th_d - agent.th_dd) / (2*pi);
    
    
    agent.th_d_f =[    agent.th(1:agent.n-1) - agent.th(2:agent.n);
                     agent.th(agent.n) - agent.th(1)-2*pi     ];
   

    % La matrice di controllo P_th è fatta così:
    % la prima riga porta semplicemente il primo drone a theta = 0
    % il blocco centrale del sistema (look_back ) muove i droni secondo 
    % l'errore rispetto al drone precedente e sucessivo 
    % l'ultima riga lascia, ovviamente R dov'è
     
    % | 0.8  0   0   0  . . . .  0.2 |   | 0.8  0   0   0  . . .  | 0.2 |
    % |  *   *   0   0  . . . .   0  |   |                        |  0  |
    % |  0   *   *   0  . . . .   0  |   |                        |  0  |
    % |  0   0   *   *  . . . .   0  | = |        look_around       |  0  |
    % |  . . . .. . . . . .*  *   0  |   |                        |  0  |
    % |  0 . . .. . . . . . . 0   1  |   |  0 . . .. . . . . .  0 |  1  |
    
    look_around = eye(agent.n) -2*diag(agent.th_err) + diag(agent.th_err(2:agent.n),-1) + diag(agent.th_err(2:agent.n),1) ;
    look_around = look_around + diag(agent.th_err(1),-(agent.n-1));
    
    
    P_th = [ 0.8     zeros(1, agent.n-1)      0.2;
             look_around(2:agent.n,:)           zeros(agent.n-1,1);
             zeros(1,agent.n)                 1               ];
    
    
    % System update
    state.rho = [agent.rho; 0];
    state.th  = [agent.th;  0];
  
    state.rho = P_rho * state.rho;
    state.th  = P_th  * state.th;
    
    agent.rho = state.rho(1:agent.n);
    agent.th  = state.th(1:agent.n);
    
    % Update cartesian coords in the two frames and get the travelled dist
    prev_pos.xr = agent.xr;
    prev_pos.yr = agent.yr;
    agent = polar2rel(agent, ugv);
    agent = rel2glob(agent, ugv);
    d_travel_x = agent.xr - prev_pos.xr;
    d_travel_y = agent.yr - prev_pos.yr;
    agent.travel_dist = sqrt(d_travel_x.*d_travel_x + d_travel_y.*d_travel_y); 

    
    
   end