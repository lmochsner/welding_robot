%% test IK

function wp = get_ik_pose(robot,cur_theta,workspace_positions) %workspace_positions 

num_points = size(workspace_positions,2);
joint_positions = zeros(robot.dof,num_points);
wp = zeros(4,num_points);

    if robot.plate == 1 %%XZ plate 
        for i = 1:num_points
        cur_point = workspace_positions(:,i);
            if i< num_points
                next_point = workspace_positions(:,i+1);
                dx = next_point(1) - cur_point(1);
                dz = next_point(3) - cur_point(3);
                alpha = atan2(dz,dx);

%                 if abs(alpha)>= pi
%                     alpha = mod(alpha,pi);
%                 end

            elseif i == num_points %guess
                next_point = cur_point;
                dx = dx; %takes previous angle
                dz = dz; %takes preious angle
                alpha = atan2(dz,dx);
            end

            pose = [cur_point;alpha];
            thetas = robot.ik_fm(pose,cur_theta);
            cur_theta = thetas;
            %joint_positions(:,i) = thetas;
            wp(:,i)  = robot.pose_eeXZ(thetas);


        end


    elseif robot.plate == 2 %% YZ Plane
        for i = 1:num_points
            cur_point = workspace_positions(:,i);
            if i< num_points
                next_point = workspace_positions(:,i+1);
                dy = next_point(2) - cur_point(2);
                dz = next_point(3) - cur_point(3);
                alpha = atan2(dz,dy);
            elseif i == num_points
                next_point = cur_point;
                dy = dy;
                dz = dz;
                alpha = atan2(dz,dy);
            end
            pose = [cur_point ; alpha];

            thetas = robot.ik_fm(pose,cur_theta);
            cur_theta = thetas; %update cur_theta
            joint_positions(:,i) = thetas;

            % find where the thetas take you
            wp(:,i) = robot.pose_eeYZ(thetas);

        end
end

