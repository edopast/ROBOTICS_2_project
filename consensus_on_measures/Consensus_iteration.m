function [consensus_average, average, last_consensus] = Consensus_iteration(n_iteration, ...
    estimated_variation, form_auv, W)

    %%  INPUT: - n_iteration: number of iteration for the consensus average;
    %          - estimated_variation: vector of estimation of UGV variation
    %                                 considering one dimension;
    %          - form_auv: structure variable of the formation of UAVs;
    %          - W: consensus matrix

    %%  OUTPUT: - consensus_average: structure where each element inside define
    %                                the temporal dynamics of estimation
    %           -  average: average by hand 
    %           - form_auv: structure variable of the formation of UAVs;
    %           - last_consensus: estimated vector at last iteration
    
     
    % we recall that our initial condition are defined in the variable
    % estimated_variation
    consensus_average = cell(n_iteration,1);
    temp_consensus = zeros(form_auv.n,1);
    % setting the initial condition
    last_consensus = estimated_variation; 

    % in this part, we are saving all the variables; totally inefficient
    % only for debugging purpose clearly and for future plots

    for i=1:n_iteration
        temp_consensus = W*last_consensus;
        consensus_average{i,1} = temp_consensus;
        last_consensus = temp_consensus;
    end

    summ = 0;
    for i=1:form_auv.n
        summ = summ + estimated_variation(i);
    end
    average = summ/form_auv.n;
    last_consensus = consensus_average{n_iteration,1};

end