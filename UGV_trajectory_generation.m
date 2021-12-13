%% UGV trajectory generation
% always start from [0 0 0]

% Experimental time [s]
T = 5;
Ts = 1e-3;

trajectory.signals.values = zeros(T/Ts,2);
trajectory.signals.dimensions = 2;
trajectory.time = zeros(1,T/Ts);

heading.signals.values = zeros(T/Ts,1);
heading.signals.dimensions = 1;
heading.time = zeros(1,T/Ts);
m = 0.3;

for i = 0:(T/Ts-1)
    trajectory.signals.values(i+1,:) = [Ts*i, m*Ts*i];
    heading.signals.values(i+1,:) = atan(m);
    trajectory.time(i+1) = Ts*i;
end

heading.time = trajectory.time;

