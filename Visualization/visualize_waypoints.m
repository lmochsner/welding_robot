%%Visualize Waypoints

function [] = check_dhp(robot,given_points,given_joints)
    
    figure
    Xg = given_points(1,:);
    Yg = given_points(2,:);
    Zg = given_points(3,:);
    plot(Yg,Zg,'o')
    hold on;
    
%     num_points = size(given_joints,2);
%     my_waypoints = zeros(3,num_points);
%     
%     for i = 1:num_points
%         theta = given_joints(:,i);
%         ee = robot.ee(theta);
%         my_waypoints(1,i) = ee(1);
%         my_waypoints(2,i) = ee(2);
%         my_waypoints(3,i) = ee(3);
%     end
%     plot3(my_waypoints(1,:),my_waypoints(2,:),my_waypoints(3,:),'o')
%     hold on;
%     
    ik_path = get_ik_pose(robot,given_points);
    plot(ik_path(2,:),ik_path(3,:),'o')
    legend('given', 'poses','inverse')
    
    figure
    plot(ik_path(2,:),ik_path(4,:),'o')

end
