%% UGV trajectory generation
% always start from [0 0 0]

% Experimental time [s]
T = 40;
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
    elseif (i>8000 && i<=9500)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = constant_desired_velocity_v;
    % turn in other direction with greater velocity
    elseif (i>9500 && i<=18000)
        w_ref.signals.values(i+1) = -1.6*constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.4*constant_desired_velocity_v;
    % straight again (higher velocity)
    elseif (i>18000 && i<=23500)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = 1.2*constant_desired_velocity_v;
    % decelerate
    elseif (i>23500 && i<=25000)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = 1.1*constant_desired_velocity_v;    
    % turn
    elseif (i>25000 && i<=33000)
        w_ref.signals.values(i+1) = -constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.7*constant_desired_velocity_v;
    else
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = constant_desired_velocity_v;
    end
    
    w_ref.time(i+1) = Ts*i;
    
end
v_ref.time = w_ref.time;

%% obtain the trajectory in x,y,theta
ugv.trajectory.signals.values = zeros(T/Ts,3);
ugv.trajectory.signals.dimensions = 3;
ugv.trajectory.time = v_ref.time;

for i = 1:(T/Ts-1)
    ugv.trajectory.signals.values(i+1,3) = ugv.trajectory.signals.values(i,3) + w_ref.signals.values(i)*Ts;  % theta
    ugv.trajectory.signals.values(i+1,1) = (ugv.trajectory.signals.values(i,1) + v_ref.signals.values(i)*cos(ugv.trajectory.signals.values(i,3))*Ts);  % x
    ugv.trajectory.signals.values(i+1,2) = (ugv.trajectory.signals.values(i,2) + v_ref.signals.values(i)*sin(ugv.trajectory.signals.values(i,3))*Ts);  % y
 
end

%% delete useless stuff
clear constant_desired_velocity_v
clear constant_desired_velocity_w

%% For Simulink Only:
pos_trajectory.signals.values = ugv.trajectory.signals.values(:,1:2);
pos_trajectory.time = ugv.trajectory.time;
head_trajectory.signals.values = ugv.trajectory.signals.values(:,3);
head_trajectory.time = ugv.trajectory.time;
