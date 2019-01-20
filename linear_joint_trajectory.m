function [ trajectory ] = linear_joint_trajectory(start_theta, goal_theta, num_points)
% linear_joint_trajectory
%
%   Returns a matrix of joint angles, where each column represents a single
%   timestamp. This matrix is length(start_theta) x num_points.
%
%   Each joint angle linearly interpolates from its start value to its end
%   value, in joint space.
%
%   'start_theta' is a column vector of joint angles
%
%   'goal_theta' is a vector of joint angles of the same dimensions as
%   start_theta
%
%   'num_points' is the number of columns in the resulting trajectory.
%
%   Hints
%   - MATLAB's linspace function can be very useful here!

% --------------- BEGIN STUDENT SECTION ----------------------------------
trajectory = zeros(size(start_theta,1), num_points);
joints = size(start_theta);
for i = 1:joints
    trajectory(i,:) = linspace(start_theta(i), goal_theta(i), num_points);
end


% --------------- END STUDENT SECTION ------------------------------------

end
