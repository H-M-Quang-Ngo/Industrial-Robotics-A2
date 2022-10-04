function RMRCMotion(robot,poseFinal,steps)
    % robot: robot object, which has MoveRobot method
    % qFinal: final joint state of the robot
    % steps : desired steps for the trajectory from current -> final Pose
    
    % safe step of time for not exceed each joint's max speed
    timestep = 0.05;
    
    % manipulability threshold
    mani_threshold = 0.0044;

    % max damping coefficient 
    damping_coefficient_MAX = 0.05;

    % get initial pose
    poseCurrent = robot.model.fkine(robot.model.getpos);
    
    pointCurrent = poseCurrent(1:3,4);
    pointFinal = poseFinal(1:3,4);
    error_displacement = norm(pointFinal - pointCurrent);
    
    count = 0;
    
    %% track the trajectory by RRMC
    while error_displacement > 0.005
    
        % current joint state 
        qCurrent = robot.model.getpos;
        
        % manipulability and Jacobian
        mani =  robot.model.maniplty(qCurrent);
        J = robot.model.jacob0(qCurrent);

        % trace the end-effector
        plot3(poseCurrent(1,4),poseCurrent(2,4),poseCurrent(3,4),'r.');
    
        % current difference to the final position
        distanceDiff = transl(poseFinal) - transl(poseCurrent);
        angleDiff = tr2rpy(poseFinal) - tr2rpy(poseCurrent);
    
        % desired spatial velocity
        u = (distanceDiff/(steps-count))/timestep;
        omega = (angleDiff/(steps-count))/timestep;
        
        % reduce singularity if exists
        if mani < mani_threshold
            damping_coefficient = (1-(mani/manithreshold)^2)/damping_coefficient_MAX;
            J_DLS = J'/(J*J'+ damping_coefficient*eye(6));    % damped least square Jacobian
            
            % joint velocity
            qd = J_DLS * [u; omega'];
        else
            % joint velocity
            qd = pinv(J) * [u; omega'];
        end

        % next position
        qNext = qCurrent + qd'*timestep;
        
        % check the validity of 'qNext':
        checkLimit = CheckJointLimit(robot.model,qNext);
        if  checkLimit <= robot.model.n
            disp(['RMRC step ',num2str(count),'. Warning: exceed joint limit at joint ', num2str(checkLimit), ...
                '. A patch using IK solution is applied!']);
    
            % replace the invalid 'qNext' by an IK solution:
            poseNext = robot.model.fkine(qNext);
            qNext = robot.model.ikcon(poseNext,robot.model.getpos);
        end

        % move robot
        robot.MoveRobot(qNext);
    
        % retrieve the current pose
        poseCurrent = robot.model.fkine(robot.model.getpos);
        pointCurrent = poseCurrent(1:3,4);
        error_displacement = norm(pointFinal - pointCurrent);
    
        count = count+1;
    end
    
    % disp(['Current error is ',num2str(1000* error_displacement),'mm.']);
end