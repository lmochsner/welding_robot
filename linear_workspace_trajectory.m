function [ trajectory ] = linear_workspace_trajectory(robot, start_theta, goal_pos, num_points)

trajectory = zeros(size(start_theta,1), num_points);

% We know the first column:
trajectory(:, 1) = start_theta;
% HINT: it may be useful to first calculate the desired workspace trajectory to
% reference in the loop below

%goal_theta = robot.inverse_kinematics(start_theta, goal_pos);
if robot.plate == 1
    start = robot.pose_eeXZ(start_theta); %this will probably have to change
elseif robot.plate ==2
    start = robot.pose_eeYZ(start_theta); %this will probably have to change
end

path = goal_pos - start; 
dpath = path/num_points;
% Find the rests:
for col = 2:num_points
    %% Fill in trajectory(:,col) here. HINT: use trajectory(:,col-1) to help!
    prev_theta = trajectory(:,col-1); %current joint angles 
    start = start + dpath; %new point to move forward to 
    trajectory(:,col) = robot.ik_fm(start,prev_theta);
% --------------- END STUDENT SECTION ------------------------------------
end

end
