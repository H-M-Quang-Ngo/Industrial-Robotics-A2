clear
close all

r = DobotMagicianwithGripper;
hold on
h = PlaceObject('Carrot.ply',[0,-0.276,0.02]);

% desired joint state at final position
qPick = [-90 45 35 -35 -90]*pi/180;

% required initial and final pose
poseCurrent = r.model.fkine(r.model.getpos);
poseFinal = r.model.fkine(qPick);

pointCurrent = poseCurrent(1:3,4);
pointFinal = poseFinal(1:3,4);
error_displacement = norm(pointFinal - pointCurrent);

% number of desired steps from initial position to final position
steps = 50;

% safe step of time for not exceed each joint's max speed
timestep = 0.05;

% manipulability threshold
mani_threshold = 0.0044;

% max damping coefficient 
damping_coefficient_MAX = 0.05;


% random collision trigger:
checkCollision = randi([1,steps+10],1,1);

count = 0;

%% track the trajectory by RRMC
% the closer the obstacle to the target, the less accurate of the
% avoidance

while error_displacement > 0.005

    % trace the end-effector
    plot3(poseCurrent(1,4),poseCurrent(2,4),poseCurrent(3,4),'r.');

    % check collision and try to avoid
    if count == checkCollision
        disp(['Collision detected at step ',num2str(count)]);

        % try to avoid that collision if possible
        if (count/steps) < 3/5
            % lift the arm
            zLift = 0.2;
            T_Lift = transl(0,0,zLift)*poseCurrent;
            RMRCMotion(r,T_Lift,40);

            % move in plane xy
            xMove = 0.08;
            yMove = 0.08;
            T_xyMove = transl(-xMove,-yMove,0)*T_Lift;
            RMRCMotion(r,T_xyMove,40);
        else
            disp('Cannot find way to avoid this collision. Stop the robot!');
            break;
        end
    end

    % current joint state 
    qCurrent = r.model.getpos;
    
    % current manipulability and Jacobian
    mani =  r.model.maniplty(qCurrent);
    J = r.model.jacob0(qCurrent);
 
    % retrieve the current pose
    poseCurrent = r.model.fkine(qCurrent);

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
    checkLimit = CheckJointLimit(r.model,qNext);
    if  checkLimit <= r.model.n
        disp(['Step ',num2str(count),'. Warning: exceed joint limit at joint ', num2str(checkLimit), ...
            '. A patch using IK solution is applied!']);

        % replace the invalid 'qNext' by an IK solution:
        poseNext = r.model.fkine(qNext);
        qNext = r.model.ikcon(poseNext,r.model.getpos);
    end

    % move robot
    r.MoveRobot(qNext);

    % retrieve the current pose
    poseCurrent = r.model.fkine(r.model.getpos);
    pointCurrent = poseCurrent(1:3,4);
    error_displacement = norm(pointFinal - pointCurrent);

    count = count+1;
end

disp(['Current error is ',num2str(1000* error_displacement),'mm.']);
