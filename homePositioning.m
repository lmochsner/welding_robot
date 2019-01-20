% homePositioning(robot, robotHardware, frequency)
%   inputs:
%       robot: object of robot class (required to do inverse kinematics)
%       robotHardware: hebi group 
%       frequency: rate at which you want to send commands to robot
%   output: 
%       trajectory: the final trajectory from initial positions to homing
%       position
% The function takes the robot from any initial position to home position
% in two steps. First step is to move the shoulder joint to safe position.
% This is done to safe position the other joints so that they do not hit
% the table while you move them to home position.

function final_traj = homePositioning(robot,robotHardware,position, frequency)

%     gains_struct_wrapper = load('gains_file.mat'); %contains gains_struct
%     gains_struct = gains_struct_wrapper.gains_struct;
%     robotHardware.set('gains', gains_struct);
%     
    homePosition = position;
    %Get initial position
    fbk = robotHardware.getNextFeedback();
    initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)
    % Move shoulder to safe location
    homingTime = 2;
    firstSafeTrajectory = repmat(initial_thetas, 1, homingTime*frequency);
    firstSafeTrajectory(2,:) = linspace(initial_thetas(2), homePosition(2), homingTime*frequency);
    

    % Move remaining joints to safe loacation
    % Get initial position
    %fbk = robotHardware.getNextFeedback();
    %initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)
    initial_thetas = firstSafeTrajectory(:,end);
    trajectory = zeros(size(firstSafeTrajectory));
    for i = 1:size(homePosition,1)
        trajectory(i,:) = linspace(initial_thetas(i), homePosition(i), homingTime*frequency);
    end
    final_traj = [firstSafeTrajectory trajectory];
    
    %command_trajectory(robot,robotHardware, firstSafeTrajectory, frequency);
    %command_trajectory(robot,robotHardware, trajectory, frequency);
end