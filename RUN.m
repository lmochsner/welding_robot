

function final_traj = traverse_trajectory_points(robot,workspace_positions)
%%
workspace_positions(1,:) = workspace_positions(1,:) - 0;
workspace_positions(2,:) = workspace_positions(2,:) - 0.01;
workspace_positions(3,:) = workspace_positions(3,:) - 0.02;

%% Initiailize Hardware
robotHardware = HebiLookup.newGroupFromNames('Robot B', {'J1', 'J2','J3','J4','J5'});
 
warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');

cmd = CommandStruct();
fbk = robotHardware.getNextFeedback();

gains_struct_wrapper = load('robotA_gains.mat'); %contains gains_struct
gains_struct = gains_struct_wrapper.gains_struct;
gains_struct.positionMinTarget(1) = -pi/4;
gains_struct.positionKp = [3 3 3 3 3];
gains_struct.positionKi = [0.5 0.5 0.5 0.5 0.5];
robotHardware.set('gains', gains_struct);


%% Initialize Variables
home_position = [0.2986;0.7837;0.9929;-0.1465;0.3738];
joints = 5;
frequency_1 = 50;
frequency_2 = 50;
frequency_3 = 50;

%% Get mapping of [x;y;z;alpha] cordinates
smooth = traj_curvspace(robot,workspace_positions);
poses = get_ik_pose(robot,home_position,smooth); %4xn matrix of points that robot needs to traverse

%% Map to Home
home_traj = homePositioning(robot,robotHardware,home_position,frequency_1); %(Resting to Home)

%% Mapping from Home to WP1
wp_1 = poses(:,1); 
num_points = 200;
goal_theta = robot.ik_fm(wp_1,home_position);
traj2 = linear_joint_trajectory(home_position,goal_theta,num_points);


%% Map from WP1 to End
num_wp = size(poses,2); 
trajectory = zeros(joints,num_wp);
cur_theta = traj2(:,end); %Theta at WP1;

for col = 1:num_wp
    goal = poses(:,col);
    next_theta = robot.ik_fm(goal,cur_theta);
    trajectory(:,col) = next_theta;
    cur_theta = next_theta;
end

final_traj = [home_traj traj2 trajectory]; %combine trajectories

%% Traverse through trajectories
    currentDir = fileparts(mfilename('fullpath'));
    logFile = robotHardware.startLog('file', fullfile(currentDir, 'repeat_waypoints'));
    
    commanded = command_trajectory(robot,robotHardware,final_traj,40);

%% Plot Theta Values 
    figure
    hold on 
    t1 = mod(trajectory(1,:),pi);
    t2 = mod(trajectory(2,:),pi);
    t3 = mod(trajectory(3,:),pi);
    t4 = mod(trajectory(4,:),pi);
    t5 = mod(trajectory(5,:),pi);
    title('Theta Values')
  

%% Plot Hebilog Outputs
robotHardware.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'repeat_waypoints.hebilog'));

% Compute workspace end effector path
n = length(hebilog.time);
x = zeros(n,1);
y = zeros(n,1);
z = zeros(n,1);
XR = workspace_positions(1,:);
YR = workspace_positions(2,:);
ZR = workspace_positions(3,:);
for i = 1:n
  ee = robot.ee(hebilog.position(i,:)');
  x(i) = ee(1);
  y(i) = ee(2);
  z(i) = ee(3);
end

% Plot path data
figure();
p1 = plot3(x, y,z, 'k-', 'LineWidth', 1);
c1 = p1.Color;
p1.Color = 'red';
hold on
p2 = plot3(XR,YR,ZR, 'k-', 'LineWidth',1);
c2 = p2.Color;
p2.Color = 'blue';
hold off
title('Plot of end effector position during trajectory');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
axis equal;

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.position, 'LineWidth', 1)
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocity, 'LineWidth', 1)
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');

%Plot Changes
figure();
plot(hebilog.time,hebilog.position,'LineWidth', 1)
xlabel('t')
ylabel('Error in Joint Values')

