classdef Robot3D
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        dhp
        h
        plate
        a_scale
    end
    
    methods
        % Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot3D(dhp,h,plate) %neglecting masses for milestone
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(dhp,2) ~= 4
               error('Invalid DH Parameter Matrix: Should be a nx4 matrix, is %dx%d.', size(dhp, 1), size(dhp, 2));
            end
            robot.dhp = dhp;
            robot.dof = size(dhp,1);
            robot.h = h;
            if ((plate ~= 1) && (plate ~= 2))
                error('need to define plate')
            end
            
            robot.plate = plate;
            robot.a_scale = 0.02;

        end
       
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        
        %% Foward Kinematics using Denivit Hartenburg Constraints
        function frames = forward_kinematics(robot, thetas)
            %thetas should be a nx1 colom where n = DOF
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(4,4,robot.dof + 1);
            n = robot.dof;
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 3x3 matrix frames(:,:,i).
            
            
            for i = 0:n %frames 00->0n
                if i == 0
                    Hi = eye(4); 
                    curr_frame_index = i+1;
                    frames(:,:,curr_frame_index) = Hi;
                else 
                    a = robot.dhp(i,1);
                    alpha = robot.dhp(i,2);
                    d = robot.dhp(i,3);
                    theta = robot.dhp(i,4)+thetas(i);
                    Hi = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                           sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                           0           sin(alpha)             cos(alpha)            d;
                           0           0                      0                     1;
                           ];

                    curr_frame_index = i+1; %ith frame is stored in the i+1 index
                    prev_frame = i;
                    frames(:,:,curr_frame_index) = frames(:,:,prev_frame) * Hi;
                end 
            end
            

            Tn_ee = [1 0 0 0;
                     0 1 0 0;
                     0 0 1 robot.h;
                     0 0 0 1;];
                 
            % Translate in z direction by length of the last link
            frames(:,:,end) = frames(:,:,end) * Tn_ee; %n2 index
            
            % adjust frame 4 (Joint 4)
            offset_4 = 0.08;
            
            T4 = [1 0 0 0;
                  0 1 0 0;
                  0 0 1 offset_4;
                  0 0 0 1;];
              
            frames(:,:,4) = frames(:,:,4) * T4;
            
            % adjust frame 5
            offset_5 = 0.025;
            
            T5 = [1 0 0 0;
                  0 1 0 0;
                  0 0 1 offset_5;
                  0 0 0 1;];
            
            frames(:,:,5) = frames(:,:,5) * T5;
            
                
            
        end
       
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
       
%% Robot End Effector Pose
        % Returns [x; y; theta] for the end effector given a set of joint
        % angles. 
        function ee = end_effector(robot, thetas)
            %Used this website: http://planning.cs.uiuc.edu/node103.html
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);

            H_0_ee = frames(:,:,end);    
         
            % Extract the components of the end_effector position and
            % orientation.
            x = H_0_ee(1,4);
            y = H_0_ee(2,4);
            z = H_0_ee(3,4);
            R= H_0_ee(1:3,1:3); %rotation matrix of end effector
           
            % Roll Pitch yaw calulations
            r11 = R(1,1);
            r33 = R(3,3);
            if (r11 ~= 0 && r33 ~=0)
                roll = atan2(R(2,1), R(1,1));
                pitch = atan2(-R(3,1), ((R(3,2))^2 + R(3,3)^2)^(1/2));
                yaw = atan2(R(3,2), R(3,3));
            else
                error('cannot find roll pitch yaw angles')
            end

            % Roll Pitch Yaw using powerpoint
%             if (R(1,3)~=0 || R(2,3)~=0) 
%                 roll = atan2(R(2,3), R(1,3));
%                 pitch = atan2(sqrt(1-R(3,3)^2),R(3,3));
%                 yaw = atan2(R(3,2), -R(3,1)); 
%             end

            % Pack them up nicely.
            ee = [x; y; z; roll; pitch; yaw];
        end
       
        % Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        %% Analytical Jacobian for each Frame
        function jacobians = jacobians(robot, thetas)
         
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            frames = robot.forward_kinematics(thetas);
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof); %6x5x7

            %End Effector
            H_0ee = frames(:,:,end);
%             T = [1 0 0 0;
%                 0 1 0 0 ;
%                 0 0 1 -.5;
%                 0 0 0 1;];
%             H_0ee = H_0ee *T;
            o_n = H_0ee(1:3,4);
            
            
            %Jacobian
            for j = 1:robot.dof %find the jacobian at the joints
                for i = 1:j %robot.dof = n
                    %ith dof rotates around i-1 frame
                    H_end = frames(:,:,j+1);%gets the
                    o_end = H_end(1:3,4);  
                    H = frames(:,:,i+1); % i+1 is actually grabbing ith frame
                    o = H * [0;0;0;1];
                    o = o(1:3);
                    z = H * [0;0;1;0];
                    z = z(1:3);
                    JV = cross(z,(o_end-o));
                    JW = z;
                    J = [JV; JW];
                    jacobians(:,i,j) = J;
                end
            end
        end
        
        %% End Effector Jacobian
        function j_ee = jacobian_ee(robot, thetas)
            
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            %j_ee = zeros(6,robot.dof);
            frames = robot.forward_kinematics(thetas);
            
            %End Effector
            H_0ee = frames(:,:,end);
            o_n = H_0ee(1:3,4);
            
            %initialize jacobian 
            n = robot.dof;
            j_ee = zeros(6,n);
            
            for i = 1:robot.dof
                H = frames(:,:,i); %this indexes to the i-1 frame which is where joint i is 
                o = H * [0;0;0;1];
                o = o(1:3);
                z = H*[0;0;1;0];
                z = z(1:3);
                JV = cross(z,(o_n-o));
                JW = z;
                J = [JV;JW];
                j_ee(:,i) = J;
            end
            
        end
        
        %% Numerical Jacobian
        
        % only need 3x5 because only have forces acting on the robot
        function jacobians = num_jacobian(robot, thetas)
            delta = 0.001;
            jacobians = zeros(3,robot.dof,robot.dof+1);
            
            for joint = 1:robot.dof
                thetas(joint) = thetas(joint) - delta; 
                f_1 = robot.fk(thetas); % 4x5x6 matrix   
                thetas(joint) = thetas(joint) + 2*delta; 
                f_2 = robot.fk(thetas); %4x5x6 matrix
                dFdj = (f_2-f_1)/(2*delta); %compute the derivative 
                
                
                num_frames = size(dFdj,3); %num_frames = 6
                
                for frame = 1: num_frames %should be 1:robot.dof+1
                    jacobians(1, joint, frame) = dFdj(1,4,frame); 
                    jacobians(2, joint, frame) = dFdj(2,4,frame);
                    jacobians(3, joint, frame) = dFdj(3,4,frame);  
                    %jacobians(4, joint, frame) = dFdj(4,4,frame);  
                end
                
            end
            
        end

%% Gravity Compensation

        function torques = gravity_comp(robot,thetas)
            %Masses of Joints and End Effector
            %need to tune the masses
            m1 = 0;
            m2 = 0;
            m3 = 0.35;
            m4 = 0.4;
            m5 = 0.35;
            mee = 0.35;
            masses = 9.81 * [m1;m2;m3;m4;m5;mee]; %includes a mass at the end effector
            J_frames = robot.num_jacobian(thetas);% (3x5x6)
            
            torques = zeros(5,1);

            for joint = 1: robot.dof %finding torques on joint 
                for frame = joint:robot.dof %Jacobian starts at 1 being the base 
                    f =  masses(frame+1) * [0;0;1];  
                    J = J_frames(:,joint,frame+1);
                    torques(joint) = torques(joint) + J' * f;
                end 
            end
        
        end
        
%% Inverse Kinematics 
        function thetas = inverse_kinematics(robot, initial_thetas, goal_position)
            %Goal_position_ x,y,z,theta (know the orientation fo the end
            %effector
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end

            if (size(goal_position, 1) ~= 6 || size(goal_position,2) ~=1)
                error('Invalid goal_position: Should be 4 length column vector, is %dx%d.', size(goal_position, 1), size(goal_position, 2));
            end

            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            thetas = initial_thetas;

            % Step size for gradient update
            step_size = 0.1;

            % Once the norm (magnitude) of the computed gradient is smaller than
            % this value, we stop the optimization
            stopping_condition = 0.00001;

            % Also, limit to a maximum number of iterations.
            max_iter = 10000;
            num_iter = 0;

% --------------- BEGIN STUDENT SECTION ----------------------------------
            % Run gradient descent optimization
            frames_ee = robot.end_effector(thetas);
            
            while (num_iter < max_iter)
                %[x;y;z;roll;pitch;yaw] goal
                frames_ee = robot.end_effector(thetas); %x,y,z,roll,pitch,yaw

                diff = frames_ee - goal_position;
                J_ee = robot.jacobian_ee(thetas);
                cost_gradient = transpose(J_ee) * diff;

                thetas = thetas - step_size * cost_gradient;
               
                % Check stopping condition, and return if it is met.
                % TODO
                if (norm(cost_gradient) < stopping_condition)
                    return 
                end
                num_iter = num_iter + 1;    
            end
% --------------- END STUDENT SECTION ------------------------------------
        end
%% Inverse Kinematics Using Matlab NonLinear Least Squares Method
function goal_angles = ik(robot, goal_position, initial_theta)
        
    options = optimset( 'algorithm', {'levenberg-marquardt',.1}, ...
                    'DerivativeCheck', 'off', ...
                    'TolX', .0000002, ...
                    'Display', 'off', ...
                    'MaxIter', 100000 );
                
    function err = my_error_function(xyz_target, robot, theta)
        actual_pos = robot.end_effector(theta);
        %% Just get the position component (modify this for position and orientation
        %% goals!)
        actual_pos = actual_pos;
        err = (xyz_target - actual_pos).^2;
    end
    
    goal_angles = lsqnonlin( @(theta) my_error_function(...
    goal_position, robot, theta), ...
    initial_theta, [], [], options );

end

%% Information for Inverse Kinematics

function pose = pose_eeXZ(robot,thetas)
    pose = zeros(4,1);
    frames = robot.fk(thetas);
    H0_ee = frames(:,:,end);
    pose(1:3,1) = H0_ee(1:3,4);
    
    r11 = -H0_ee(1,1);
    r31 = -H0_ee(3,1);
    %find angle of x-axis of end-effector to x-axis of base frame

    alpha = atan2(r31,r11);
    %alpha = 0;
    
    pose(4,1) = alpha;

end

function pose = pose_eeYZ(robot,thetas)
    pose = zeros(4,1);
    frames = robot.fk(thetas);
    H0_ee = frames(:,:,end);
    pose(1:3,1) = H0_ee(1:3,4);
    
    r32 = -H0_ee(3,2);
    r22 = -H0_ee(2,2);
    
    alpha = atan2(r32,r22);
 

    pose(4,1) = alpha;
end


%% Inverse Kinematics Using Matlab Fmincon
function goal_thetas = ik_fm(robot, goal_position, initial_thetas)  
    
    function cost = nested_cost(x)
        weights = [1;0.5;1;0.02];
        if robot.plate == 1         
             cur_pos = robot.pose_eeXZ(x);            
             error = cur_pos - goal_position;           
             error_pose =  abs(cur_pos(4) - goal_position(4));
              if error_pose >= pi          
                  error(4) =2* pi - error_pose;
              end
             %error(4) = error_pose;
             cost = 1/2 * norm(weights .* error);
        elseif robot.plate == 2
            error = robot.pose_eeYZ(x) - goal_position;
            if error_pose >= pi          
                error(4) = 2*pi - error_pose;
            end
            cost = 1/2 * norm(weights .* error); 
        end  
    end 
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    %bounds of goal_theta 
    lb = -pi *[1;1;1;1;1]; %impose limit to joint 3
    ub = pi * [1;1;1;1;1]; 
    
    options = optimoptions('fmincon','Display','off','Algorithm','sqp','OptimalityTolerance', 0.000005);
    
    goal_thetas = fmincon(@nested_cost,initial_thetas,A,b,Aeq,beq,lb,ub, [],options);
    

end


    end
end