
function traj = traj_curvspace(robot,waypoints)

    Xo = waypoints(1,:);
    Yo = waypoints(2,:);
    Zo = waypoints(3,:);


    p = [Xo',Yo',Zo'];
    N = 600; %600
    q = curvspace(p,N);
    traj = q';
    
    if robot.plate == 1
        %figure
        %hold on
        %plot(waypoints(1,:), waypoints(3,:),'o')
        %plot(traj(1,:), traj(3,:))
        %hold off
        
        %figure
        %plot3(traj(1,:),traj(2,:), traj(3,:))

end
