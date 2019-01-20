
function [] = animate_trajectory(robot,final_traj)

    ee = zeros(4,size(final_traj,2));
    for i = 1: size(final_traj,2)
        t = final_traj(:,i);
        P = robot.pose_eeXZ(t);
        ee(:,i) = P;
    end
    
    figure
    plot3(ee(1,:), ee(2,:), ee(3,:),'o')
    
    figure
    title('Animate Trajectory')
    subplot(3,1,1);
    plot(ee(1,:), ee(3,:),'o');
    xlabel('X(m)')
    ylabel('Z(m)')
    subplot(3,1,2);
    plot(ee(1,:),ee(4,:),'o');
    xlabel('X(m)')
    ylabel('Alpha')
    %plot(ee(1,:), final_traj(2,:),'o');
    %plot(poses(1,:), poses(3,:),'o');
    subplot(3,1,3)
    t1 = final_traj(1,:);
    t2 = final_traj(2,:);
    t3 = final_traj(3,:);
    t4 = final_traj(4,:);
    t5 = final_traj(5,:);
    xlabel('X(m)')
    ylabel('\theta')
    hold on
    plot(ee(1,:),t1,'o');
    plot(ee(1,:),t2,'o');
    plot(ee(1,:),t3,'o');
    plot(ee(1,:),t4,'o');
    plot(ee(1,:),t5,'o');
    yticks([-pi -pi/2 -pi/4 0 pi/4 pi/2 pi])
    legend('theta_1','theta_2;','theta_3','theta_4','theta_5')
   

    
   
    

end
