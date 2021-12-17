%% UGV trajectory generation
% always start from [0 0 0]

% Experimental time [s]
T = 20;
Ts = 1e-3;

%% linear and angular desired velocities
w_ref.signals.values = zeros(T/Ts,1);
w_ref.signals.dimensions = 1;
w_ref.time = zeros(T/Ts,1);

v_ref.signals.values = zeros(T/Ts,1);
v_ref.signals.dimensions = 1;
v_ref.time = zeros(T/Ts,1);

for i = 1:(T/Ts-1)
    if (i < T/(2*Ts))
        w_ref.signals.values(i+1) = 0.2;
    else
        w_ref.signals.values(i+1) = -0.2;
    end
    v_ref.signals.values(i+1) = 1;
    w_ref.time(i+1) = Ts*i;
end
v_ref.time = w_ref.time;

% obtain the trajectory in x,y,theta
pos_trajectory.signals.values = zeros(T/Ts,2);
head_trajectory.signals.values = zeros(T/Ts,1);
pos_trajectory.signals.dimensions = 2;
head_trajectory.signals.dimensions = 1;
pos_trajectory.time = v_ref.time;
head_trajectory.time = v_ref.time;

for i = 1:(T/Ts-1)
    head_trajectory.signals.values(i+1,1) = head_trajectory.signals.values(i,1) + w_ref.signals.values(i)*Ts;  % theta
    pos_trajectory.signals.values(i+1,1) = pos_trajectory.signals.values(i,1) + v_ref.signals.values(i)*cos(head_trajectory.signals.values(i,1))*Ts;  % x
    pos_trajectory.signals.values(i+1,2) = pos_trajectory.signals.values(i,2) + v_ref.signals.values(i)*sin(head_trajectory.signals.values(i,1))*Ts;  % x
end

plot(head_trajectory.time, head_trajectory.signals.values)
figure(2)
plot(pos_trajectory.signals.values(:,1), pos_trajectory.signals.values(:,2))