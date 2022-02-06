function [W, G] = graph_setup(form_auv)
    %% INPUT: the formation of UAVs; the variable that is exploited here 
    % only related to the number of UAVs that are actually entered into the 
    % formation
    
    %% OUTPUT: the consensus matrix exploited for average consensus; 
    % we exploit Metropolis definition of the matrix
    % in our case the sensor network consist in n different agent, where each 
    % agent is linked with its two adiacent agent that are described in 
    % the formation
    
    % initilize the nodes; the adjacency matrix is 
    % A = adjacency matrix
    % implementation of the adjacency matrix wrt number of drones in formation
    A = zeros(form_auv.n,form_auv.n);
    for i = 1: form_auv.n
        for j = 1: form_auv.n
            if i == 1
                A(i,i+1) = 1;
                A(i, form_auv.n) = 1;
            elseif i == form_auv.n
                A(i,1) = 1;
                A(i,form_auv.n-1) = 1;
            else
                A(i,i-1) = 1;
                A(i,i+1) = 1;
            end
        end
    end
    
    G = graph(A);
    
    
    % now we define the elements inside the "Metropolis matrix"
    % for our graph, we always have that max(di, dj) = 2, since all UAVs are 
    % connected with the two nearest UAVs in the circular formation
    
    % here we are using off_diag_element = 1/(1+max(di,dj)), however
    % since all the degree in the network are equal to two, then we set
    % directly 2 instead of max(di, dj)
    
    off_diag_element = 1/(1+2);
    
    % W represent the weight of the metropolis matrix 
    W = A;  % we start from the adjacency matrix for simplicity
    W(W==1) = off_diag_element;
    diag_element = zeros(form_auv.n,1);
    
    for i=1 : form_auv.n
        diag_element(i) = 1-2/3; % from metropolis matrix inspection
    end                          % for the same reason of (1/2) above;
                                 % we always have (2/3) as sum on a row
    for i=1:form_auv.n
        W(i,i) = diag_element(i);
    end
    
end

