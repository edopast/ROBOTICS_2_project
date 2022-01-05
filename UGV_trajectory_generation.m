%% UGV trajectory generation
% always start from [0 0 0]

% Experimental time [s]
T = 30;
Ts = 1e-3;

%% linear and angular desired velocities
constant_desired_velocity_v = 0.36;
constant_desired_velocity_w = 0.36;

% define 2 vectors holding, for each ms, the wanted angular and linear
% velocities
w_ref.signals.values = zeros(T/Ts,1);
w_ref.signals.dimensions = 1;
w_ref.time = zeros(T/Ts,1);

v_ref.signals.values = zeros(T/Ts,1);
v_ref.signals.dimensions = 1;
v_ref.time = zeros(T/Ts,1);

for i = 1:(T/Ts-1)
    
    % small initial turn
    if (i <= 1000)
        w_ref.signals.values(i+1) = constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.8*constant_desired_velocity_v;
    % then go straight
    elseif (i>1000 && i<=2000)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = constant_desired_velocity_v;
    % then turn again
    elseif (i>2000 && i<=8000)
        w_ref.signals.values(i+1) = constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.8*constant_desired_velocity_v;
    % go straight a little bit
    elseif (i>8000 && i<=10000)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = constant_desired_velocity_v;
    % turn in other direction with greater velocity
    elseif (i>10000 && i<=21000)
        w_ref.signals.values(i+1) = -1.3*constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.5*constant_desired_velocity_v;
    % straight again (higher velocity)
    elseif (i>21000 && i<=25000)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = 1.5*constant_desired_velocity_v;
    % turn
    else
        w_ref.signals.values(i+1) = -constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.7*constant_desired_velocity_v;
    end
    
    w_ref.time(i+1) = Ts*i;
    
end
v_ref.time = w_ref.time;

%% obtain the trajectory in x,y,theta
pos_trajectory.signals.values = zeros(T/Ts,2);
head_trajectory.signals.values = zeros(T/Ts,1);
pos_trajectory.signals.dimensions = 2;
head_trajectory.signals.dimensions = 1;
pos_trajectory.time = v_ref.time;
head_trajectory.time = v_ref.time;

for i = 1:(T/Ts-1)
    head_trajectory.signals.values(i+1,1) = head_trajectory.signals.values(i,1) + w_ref.signals.values(i)*Ts;  % theta
    pos_trajectory.signals.values(i+1,1) = (pos_trajectory.signals.values(i,1) + v_ref.signals.values(i)*cos(head_trajectory.signals.values(i,1))*Ts);  % x
    pos_trajectory.signals.values(i+1,2) = (pos_trajectory.signals.values(i,2) + v_ref.signals.values(i)*sin(head_trajectory.signals.values(i,1))*Ts); 
 
end

% % error
% for i = 1:(T/Ts)
%     pos_trajectory.signals.values(i,1) = pos_trajectory.signals.values(i,1) + 0.4;
%     pos_trajectory.signals.values(i,2) = pos_trajectory.signals.values(i,2) + 0.1;
% end