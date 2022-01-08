function [Trajectory] = computeUAVTrajectory(Pose,NextPose,time)
% This function computes a possibly smooth trajectory to be given to the
% UAV: starting from the Pose of the UAV, and given the NextPose and the
% time required to reach that point, the trajectory to follow is generated
% in order to avoid abrupt motions. Pose and NextPose must be 1x4 vectors
% with: [x y z yaw], while time must be expressed in seconds.
% The output is a Nx5 matrix where N is the number of waypoints (1 for each
% millisecond) and in the 4 columns we have [x y z yaw time]. Desired pitch
% and roll angles are always computed by the model.

time_vector= 0.001:0.001:time;
Trajectory(:,5)= time_vector.';

% check if yaw angle turning velocity is too big
if ( abs(NextPose(4) - Pose(4))/time > 1.4)  % more than 1.4 rad/s
    if NextPose(4)>Pose(4)
        NextPose(4) = 1.4*time + Pose(4);
    else
        NextPose(4) = -1.4*time + Pose(4);
    end
end

% check if z velocity is too big
if ( abs(NextPose(3) - Pose(3))/time > 2)  % more than 2 m/s
    if NextPose(3)>Pose(3)
        NextPose(3) = 2*time + Pose(3);
    else
        NextPose(3) = -2*time + Pose(3);
    end
end

% check if x,y velocity are too big
if ( abs(NextPose(1) - Pose(1))/time > 1)  % more than 1 m/s
    if NextPose(1)>Pose(1)
        NextPose(1) = 1*time + Pose(1);
    else
        NextPose(1) = -1*time + Pose(1);
    end
end

if ( abs(NextPose(2) - Pose(2))/time > 1)  % more than 1 m/s
    if NextPose(2)>Pose(2)
        NextPose(2) = 1*time + Pose(2);
    else
        NextPose(2) = -1*time + Pose(2);
    end
end

% use a convex combination
for i=1:size(time_vector,2)
    weight = i/size(time_vector,2);
    Trajectory(i,1) = (1-weight)*(Pose(1)) + (weight)*(NextPose(1));
    Trajectory(i,2) = (1-weight)*(Pose(2)) + (weight)*(NextPose(2));
    Trajectory(i,3) = (1-weight)*(Pose(3)) + (weight)*(NextPose(3));
    Trajectory(i,4) = (1-weight)*(Pose(4)) + (weight)*(NextPose(4));
end

end

