%% UAV trajectory generation
% always start from [0 0 0]

% Experimental time [s]
T = 55;
Ts = 1e-3;

%% desired position trajectory + desired yaw angle to track

p_ref.signals.values = zeros(T/Ts, 3);  % [ x , y , z ]
p_ref.signals.dimensions = 3;
p_ref.time = zeros(T/Ts,1);

yaw_ref.signals.values = zeros(T/Ts, 1);  % [ \psi ]
yaw_ref.signals.dimensions = 1;
yaw_ref.time = zeros(T/Ts,1);

for i = 1:(T/Ts-1) %take off (let z be just 1 m)
    p_ref.signals.values(i+1, 3) = 1; % z
    p_ref.signals.values(i+1, 1) = 0; % x
    p_ref.signals.values(i+1, 2) = 0; % y
    % define entry for time
    p_ref.time(i+1) = Ts*i;
    yaw_ref.time(i+1) = Ts*i;
end

for i = 2:size(UGVRealPose.Pose.time,1)
    
    % find the current time of the UGV trajectory simulation
    current_time = UGVRealPose.Pose.time(i-1);
    % retrieve current_time on the UAV reference trajectory
    index = find(p_ref.time==current_time);
    
    % find the next time of the UGV trajectory simulation
    next_time = UGVRealPose.Pose.time(i);
    
    % run function. NOTE: NextPose has a 4 m offset from starting
    % position!
    Pose=[p_ref.signals.values(index,1) p_ref.signals.values(index,2) p_ref.signals.values(index,3) yaw_ref.signals.values(index,1)];
    NextPose=[UGVRealPose.Pose.signals.values(i,1)+2 UGVRealPose.Pose.signals.values(i,2)+2 1 UGVRealPose.Pose.signals.values(i,3)+pi];
    
    
    Trajectory = computeUAVTrajectory(Pose,NextPose,round(next_time-current_time,3));
    
    % substitute computed values
    for j=1:size(Trajectory,1)
        Jindex = find((p_ref.time<=Trajectory(j,5)+current_time+0.0005).*(p_ref.time>=Trajectory(j,5)+current_time-0.0005));
        p_ref.signals.values(Jindex,1) = Trajectory(j,1);
        p_ref.signals.values(Jindex,2) = Trajectory(j,2);
        p_ref.signals.values(Jindex,3) = Trajectory(j,3);
        yaw_ref.signals.values(Jindex,1) = Trajectory(j,4);
    end
end

%% old code: use in case the code above doesn't work

% for i = 2:(T/Ts-1)
%     % try to increment the number of points (UGVRealPose has 100 points a
%     % sec, while we generate 1000 points a sec, with a convex combination)
%     j = floor(i*0.1);
%     prevj = j-1;
%     if prevj<1
%         p_ref.signals.values(i+1, 1) = p_ref.signals.values(i, 1);
%         p_ref.signals.values(i+1, 2) = p_ref.signals.values(i, 2);
%         yaw_ref.signals.values(i+1, 1) = yaw_ref.signals.values(i, 1);
%         continue;
%     end
%     %p_ref.signals.values(i+1, 1) = ugv.trajectory.signals.values(i,1);
%     %p_ref.signals.values(i+1, 2) = ugv.trajectory.signals.values(i,1);
%     p_ref.signals.values(i+1, 1) = (1-i*0.1+j)*UGVRealPose.Pose.signals.values(prevj,1) + (i*0.1-j)*UGVRealPose.Pose.signals.values(j,1);
%     p_ref.signals.values(i+1, 2) = (1-i*0.1+j)*UGVRealPose.Pose.signals.values(prevj,2) + (i*0.1-j)*UGVRealPose.Pose.signals.values(j,2);
%     
%     % yaw angle
%     yaw_ref.signals.values(i+1,1) = (UGVRealPose.Pose.signals.values(j,3));
% end

%% plot

%plot3(p_ref.signals.values(:,1), p_ref.signals.values(:,2), p_ref.signals.values(:,3))
%figure(3)
%plot(yaw_ref.time, yaw_ref.signals.values)