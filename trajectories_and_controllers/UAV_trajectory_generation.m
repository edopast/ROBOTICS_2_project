%% UAV trajectory generation
% always start from [0 0 0]

% Experimental time [s]
T = 40;
Ts = 1e-3;

%% desired position trajectory + desired yaw angle to track

p_ref.signals.values = zeros(T/Ts, 3);  % [ x , y , z ]
p_ref.signals.dimensions = 3;
p_ref.time = zeros(T/Ts,1);

yaw_ref.signals.values = zeros(T/Ts, 1);  % [ \psi ]
yaw_ref.signals.dimensions = 1;
yaw_ref.time = zeros(T/Ts,1);

for i = 1:(T/Ts-1)
    p_ref.signals.values(i+1, 3) = 1; % z
    p_ref.signals.values(i+1, 1) = 1;
    p_ref.signals.values(i+1, 2) = 0;
    yaw_ref.signals.values(i+1, 1) = 0;
    p_ref.time(i+1) = Ts*i;
    yaw_ref.time(i+1) = Ts*i;
end

for i = round((T/Ts)/3):(T/Ts-1)
    p_ref.signals.values(i+1, 1) = 1*cos(0.0005*(i-round((T/Ts)/3)));    % each millisecond: 1.8 degrees in x and y
    p_ref.signals.values(i+1, 2) = 1*sin(0.0005*(i-round((T/Ts)/3)));
    %yaw_ref.signals.values(i+1, 1) = 0.0005*(i-((T/Ts)/2));
    p_ref.time(i+1) = Ts*i;
    yaw_ref.time(i+1) = Ts*i;
end

%plot3(p_ref.signals.values(:,1), p_ref.signals.values(:,2), p_ref.signals.values(:,3))
%figure(2)
%plot(yaw_ref.time, yaw_ref.signals.values)