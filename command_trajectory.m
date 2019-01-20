
% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.
function error = command_trajectory( robot, robotHardware, trajectory, frequency)
  %% Setup reusable structures to reduce memory use in loop
  %tmpFbk = robotHardware.getNextFeedback();
    cmd = CommandStruct();
%     gains_struct_wrapper = load('robotA_gains.mat'); %contains gains_struct
%     
%     gains_struct = gains_struct_wrapper.gains_struct;
%     gains_struct.positionMinTarget(1) = -pi/4;
%     robotHardware.set('gains', gains_struct);

  error = zeros(size(trajectory,1),size(trajectory,2));
  % Compute the velocity numerically
  trajectory_vel = diff(trajectory, 1,2)/100;
  joint_error = zeros(5,size(trajectory,2));
  % Command the trajectory
  for i = 1:(size(trajectory, 2) - 1)
    
    % Send command to the robot (the transposes on the trajectory
    % points turns column into row vector for commands).
    cmd.position = trajectory(:,i)';
    cmd.velocity = trajectory_vel(:,i)';
    
    %Torques for gravity comp
    fbk = robotHardware.getNextFeedback();
    cur_thetas = fbk.position';
    num_torques = robot.gravity_comp(cur_thetas);
    cmd.torque = num_torques';
    
    
    robotHardware.set(cmd);
    error(:,i)= trajectory(:,i) - fbk.position'; 
    joint_error(:,i) = trajectory(:,i) - fbk.position'; 
    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
  end
  % Send the last point, with a goal of zero velocity.
  cmd.position = trajectory(:,end)';
  cmd.velocity = zeros(1, size(trajectory, 1));
  %Numerical Torque Calculations
  tmpfbk = robotHardware.getNextFeedback();
  cur_thetas = fbk.position';
  num_torques = robot.gravity_comp(cur_thetas);
  cmd.torque = num_torques';
   robotHardware.set(cmd);
   
   figure
   hold on
   plot(joint_error(1,:));
   plot(joint_error(2,:));
   plot(joint_error(3,:));
   plot(joint_error(4,:));
   plot(joint_error(5,:));
   legend('theta_1','theta_2;','theta_3','theta_4','theta_5')
   ylabel('Joint Error')
   title('Joint Error 1/4 Frequency');
   
  %error = tmpfbk.position' - trajectory(:,end);
end