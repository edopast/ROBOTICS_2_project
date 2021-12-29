function [auv] = consensusPolarFormation(auv, ugv, f_radius, new)
%% function that compute the position at the next iteration of the auvs with polar consensus

    
    K.rho = 0.1;
    
    % Retrieve auvs positions wrt robot frame
    auv = glob2rel(auv, ugv);
    
    % Retrieve polar coords
    auv = rel2polar(auv, ugv);
    
    % Normalized distance error
    auv.d_err = auv.rho - ones(auv.n , 1) * f_radius;
    auv.d_err_norm = auv.d_err./auv.rho;
    
    % Control matrix for rho
    P_rho = [eye(auv.n)-diag(K.rho*auv.d_err)   K.rho*auv.d_err;
             zeros(1,auv.n)                     1               ];
    
    % Consensus matrix per theta :
    % calcolare differenza delta (th_d) tra il drone e quello precedente;
    % fare average consensus su questo delta, in modo da trovare la media
    % dei delta che ogni drone deve avere rispetto al suo precedente 
    % e sucessivo per essere equamente distribuiti
    
    % In questo consenso consderiamo solo i droni, l'UGV non serve
    
    auv.th_d =[   auv.th(1)+2*pi - auv.th(auv.n);
                    auv.th(2:auv.n) - auv.th(1:auv.n-1)];

    % il vettore è costruito così: (v1' = theta(v1) +2pi)
    % | v1'-vn|    | th_d1 |
    % | v2-v1 |    | th_d2 |
    % | v3-v2 |  = | th_d3 |
    % | ..... |    | ..... |
    % | ..... |    | ..... |
    % |vn-vn-1|    | th_dn |
    
    % Definisco la matrice di average consensus:
    coeff_th_d = 1/5 * ones(1, auv.n);
    
    % matrice di consenso per delta theta:
    % se stesso
    P_th_d =  diag(coeff_th_d);
    % drone prima
    P_th_d =  P_th_d + diag(coeff_th_d(2:auv.n),-1)  + diag(coeff_th_d(1),auv.n-1); 
    % drone dopo       
    P_th_d =  P_th_d + diag(coeff_th_d(2:auv.n),+1)  + diag(coeff_th_d(1),-(auv.n-1));
    % due droni prima
    P_th_d =  P_th_d + diag(coeff_th_d(3:auv.n),-2)  + diag(coeff_th_d(1:2),auv.n-2); 
    % due droni dopo       
    P_th_d =  P_th_d + diag(coeff_th_d(3:auv.n),+2)  + diag(coeff_th_d(1:2),-(auv.n-2));
    
    % update delta -> obtain delta "desired" with consensus
    if(new) % calcola th_dd sulla base delle misurazioni attuali
        auv.th_dd = P_th_d * auv.th_d;
        disp("calcolato per la prima volta");
    else % calcola th_dd sulla base dei precedenti th_dd
        auv.th_dd = P_th_d * auv.th_dd;
        disp("calcolatosulle precedenti");
    end
    
    
    % Ora si calcola l'errore di theta definito come:
    % delta_attuale - delta_desired -> poi normalizzato
    auv.th_err = 0.5*(auv.th_d - auv.th_dd) / (2*pi);
    
    
    auv.th_d_f =[    auv.th(1:auv.n-1) - auv.th(2:auv.n);
                     auv.th(auv.n) - auv.th(1)-2*pi     ];
   

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
    
    look_around = eye(auv.n) -2*diag(auv.th_err) + diag(auv.th_err(2:auv.n),-1) + diag(auv.th_err(2:auv.n),1) ;
    look_around = look_around + diag(auv.th_err(1),-(auv.n-1));
    
    
    P_th = [ 0.8     zeros(1, auv.n-1)      0.2;
             look_around(2:auv.n,:)           zeros(auv.n-1,1);
             zeros(1,auv.n)                 1               ];
    
    
    % System update
    state.rho = [auv.rho; 0];
    state.th  = [auv.th;  0];
  
    state.rho = P_rho * state.rho;
    state.th  = P_th  * state.th;
    
    auv.rho = state.rho(1:auv.n);
    auv.th  = state.th(1:auv.n);
    
    % Update cartesian coords in the two frames
    auv = polar2rel(auv, ugv);
    auv = rel2glob(auv, ugv);
end