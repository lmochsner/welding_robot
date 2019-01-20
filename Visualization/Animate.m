function [] = Animate(robot, joint_positions, frequency)
figure
hold on
ee_points = zeros(3, size(joint_positions,2));
for i = 1:size(joint_positions, 2)
    frames = robot.fk(joint_positions(:, i));
    num_frames = size(frames, 3);
    
    pose = robot.pose_eeXZ(joint_positions(:, i));
    
    x = reshape(frames(1, 4, :), [num_frames, 1]);
    y = reshape(frames(2, 4, :), [num_frames, 1]);
    z = reshape(frames(3, 4, :), [num_frames, 1]);

    ee_points(:,i) = [x(end)', y(end)', z(end)'];
    clf
    
    scatter3(ee_points(1,1:i)', ee_points(2,1:i)', ee_points(3,1:i)');
    hold on
    plot3(x,y,z);
    axis([-1.0 0.5 -0.5 0.7 -0.1 0.5]);
    
    drawnow
    pause(1/frequency);
end
end

