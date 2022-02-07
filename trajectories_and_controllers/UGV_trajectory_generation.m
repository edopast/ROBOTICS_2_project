%% UGV trajectory generation

% Experimental time [s]
T = 55;
Ts = 1e-3;

%% linear and angular desired velocities
constant_desired_velocity_v = 0.35;
constant_desired_velocity_w = 0.35;

% define 2 vectors holding, for each ms, the wanted angular and linear
% velocities
w_ref.signals.values = zeros(T/Ts,1);
w_ref.signals.dimensions = 1;
w_ref.time = zeros(T/Ts,1);

v_ref.signals.values = zeros(T/Ts,1);
v_ref.signals.dimensions = 1;
v_ref.time = zeros(T/Ts,1);

for i = 1:(T/Ts-1)
    if (i <= 15000)
        w_ref.signals.values(i+1) = 0.0;
        v_ref.signals.values(i+1) = 0.0;
    % small initial turn
    elseif (i>15000 && i<=16000)
        w_ref.signals.values(i+1) = constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.8*constant_desired_velocity_v;
    % then go straight
    elseif (i>16000 && i<=17000)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = constant_desired_velocity_v;
    % then turn again
    elseif (i>17000 && i<=23000)
        w_ref.signals.values(i+1) = constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.8*constant_desired_velocity_v;
    % go straight a little bit
    elseif (i>23000 && i<=24500)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = constant_desired_velocity_v;
    % turn in other direction with greater velocity
    elseif (i>24500 && i<=33000)
        w_ref.signals.values(i+1) = -1.6*constant_desired_velocity_w;
        v_ref.signals.values(i+1) = 0.4*constant_desired_velocity_v;
    % straight again (higher velocity)
    elseif (i>33000 && i<=40000)
        w_ref.signals.values(i+1) = 0;
        v_ref.signals.values(i+1) = 1.1*constant_desired_velocity_v;    
    % turn
    elseif (i>40000 && i<=48000)
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
% uncomment the next lines if you want to have a small initial offset
% in position / orientation:
% ugv.trajectory.signals.values(1,1) = 0.05;
% ugv.trajectory.signals.values(1,2) = 0.05;
% ugv.trajectory.signals.values(1,3) = 0.02;

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
