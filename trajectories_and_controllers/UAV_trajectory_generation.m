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

for i = 1:(T/Ts-1) %take off (1.5 seconds, then let z be just 1 m)
    p_ref.signals.values(i+1, 3) = 1; % z
    p_ref.signals.values(i+1, 1) = 0; % x
    p_ref.signals.values(i+1, 2) = 0; % y
    p_ref.time(i+1) = Ts*i;
    yaw_ref.time(i+1) = Ts*i;
end


for i = 2:(T/Ts-1)
    % try to increment the number of points (UGVRealPose has 100 points a
    % sec, while we generate 1000 points a sec, with a convex combination)
    j = floor(i*0.1);
    prevj = j-1;
    if prevj<1
        p_ref.signals.values(i+1, 1) = p_ref.signals.values(i, 1);
        p_ref.signals.values(i+1, 2) = p_ref.signals.values(i, 2);
        yaw_ref.signals.values(i+1, 1) = yaw_ref.signals.values(i, 1);
        continue;
    end
    %p_ref.signals.values(i+1, 1) = ugv.trajectory.signals.values(i,1);
    %p_ref.signals.values(i+1, 2) = ugv.trajectory.signals.values(i,1);
    p_ref.signals.values(i+1, 1) = (1-i*0.1+j)*UGVRealPose.Pose.signals.values(prevj,1) + (i*0.1-j)*UGVRealPose.Pose.signals.values(j,1);
    p_ref.signals.values(i+1, 2) = (1-i*0.1+j)*UGVRealPose.Pose.signals.values(prevj,2) + (i*0.1-j)*UGVRealPose.Pose.signals.values(j,2);
    
    % yaw angle
    yaw_ref.signals.values(i+1,1) = (UGVRealPose.Pose.signals.values(j,3));
end

%plot3(p_ref.signals.values(:,1), p_ref.signals.values(:,2), p_ref.signals.values(:,3))
%figure(3)
%plot(yaw_ref.time, yaw_ref.signals.values)