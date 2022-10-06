% clear
% close all

carrot = Veggie('Carrot.ply',transl(0,-0.276,0.02),[255,128,0]/255);
r = DobotMagicianwithGripper;

view([34.58 27.20])

pause()

xlim([-0.65 3.56])
ylim([-2.14 1.35])
zlim([-0.05 2.36])
view([17.70 24.49])

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
%checkCollision = randi([1,steps+10],1,1);
checkCollision = 24;

count = 0;
pause();
%% track the trajectory by RRMC
% the closer the obstacle to the target, the less accurate of the
% avoidance

while error_displacement > 0.003

    % trace the end-effector
    % plot3(poseCurrent(1,4),poseCurrent(2,4),poseCurrent(3,4),'r.');

    % check collision and try to avoid
    if count == checkCollision
        disp(['Collision detected at step ',num2str(count)]);

        % try to avoid that collision if possible
        if (count/steps) < 3/5
            % lift the arm
            zLift = 0.2;
            T_Lift = transl(0,0,zLift)*poseCurrent;
            RMRCMotion(r,T_Lift,30);

            % move in plane xy
            xMove = 0.08;
            yMove = 0.08;
            T_xyMove = transl(-xMove,-yMove,0)*T_Lift;
            RMRCMotion(r,T_xyMove,30);
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

posePick_mid = r.model.fkine(r.model.getpos);

%% Lower the arm to pick the object 

if error_displacement <= 0.003
    posePick = transl(0,0,-0.04)*posePick_mid;
    
    r.MoveGripper(25);
    RMRCMotion(r,posePick,20);
    r.MoveGripper(24);

%% Bring it to the start position
    RMRCMotion(r,posePick_mid,20,carrot);
    RMRCMotion(r,r.model.fkine(r.defaultJoint),50,carrot);
end

%% Replace the carrot with carrot pieces

carrotPose = carrot.pose;
delete(carrot.mesh_h);
delete(carrot);
hold on

for i = 1:6
    carrotPiece(i) = Veggie(['CarrotPiece_',num2str(i),'.ply'],carrotPose);
end


