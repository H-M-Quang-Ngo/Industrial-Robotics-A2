clear
% close all

% create the environment
CreateEnvironment();

% type of the veggie to cut
veggie_Type = 'Cucumber';

switch veggie_Type
    case 'Carrot'
        veggie_Color = [255,128,0]/255;

        % initial position above the veggie
        veggie_Pose = transl(0,-0.276,0.02);

        % desired joint state for the dobot at the upper picking position
        qPick_Dobot = [-90 45 35 -35 -90]*pi/180;

    case 'Cucumber'
        veggie_Color = [30,170,30]/255;
        veggie_Pose = transl(0.01,-0.23,0.02);
        qPick_Dobot = [-88.86 30.197 60.7302 -45.93 -90]*pi/180;
end

veggie = Veggie([veggie_Type,'.ply'],veggie_Pose,veggie_Color);

% call the two robots
robot1 = DobotMagicianwithGripper;
robot2 = Dorna2Robot;

% adjust the initial position of the Dorna
robot2.model.base = transl([0.77 0 0]) * trotz(pi);
robot2.MoveRobot(robot2.model.getpos);

% minimum max speed of the two robot
speedMax_Dobot = deg2rad(320);
speedMax_Dorna = deg2rad(450);

%%  Set up for Dobot
view([34.58 27.20])

pause()

xlim([-0.65 3.56])
ylim([-2.14 1.35])
zlim([-0.05 2.36])
view([17.70 24.49])

% default poses of the two robot
pose_Default_Dobot = robot1.model.fkine(robot1.jointDefault);
pose_Default_Dorna = robot2.model.fkine(robot2.jointDefault);

% required initial and the pose above the veggie
poseCurrent_Dobot = robot1.model.fkine(robot1.model.getpos);
poseAboveVeggie_Dobot = robot1.model.fkine(qPick_Dobot);

pointCurrent_Dobot = poseCurrent_Dobot(1:3,4);
pointFinal_Dobot = poseAboveVeggie_Dobot(1:3,4);
error_displacement_dobot = norm(pointFinal_Dobot - pointCurrent_Dobot);

% number of desired steps from initial position to final position
steps = 50;

% safe step of time for not exceed each joint's max speed
timestep = 0.05;

% manipulability threshold
mani_threshold = 0.0044;

% max damping coefficient 
damping_coefficient_MAX = 0.5;


% random collision trigger:
%checkCollision = randi([1,steps+10],1,1);
checkCollision = 14;

count = 0;
pause();

disp("Dobot starts!");

%% Dobot tracks the trajectory by RRMC to reach the veggie
% the closer the obstacle to the target, the less accurate of the
% avoidance

while error_displacement_dobot > 0.005

    % trace the end-effector
    % plot3(poseCurrent(1,4),poseCurrent(2,4),poseCurrent(3,4),'r.');

    % check collision and try to avoid
    if count == checkCollision
        disp(['Collision detected at step ',num2str(count)]);

        % try to avoid that collision if possible
        if (count/steps) < 3/5
            % lift the arm
            zLift = 0.2;
            T_Lift = transl(0,0,zLift)*poseCurrent_Dobot;
            RMRCMotion(robot1,T_Lift,25,speedMax_Dobot);

            % move in plane xy
            xMove = 0.08;
            yMove = 0.08;
            T_xyMove = transl(-xMove,-yMove,0)*T_Lift;
            RMRCMotion(robot1,T_xyMove,25,speedMax_Dobot);
        else
            disp('Cannot find way to avoid this collision. Stop the robot!');
            break;
        end
    end

    % current joint state 
    qCurrent = robot1.model.getpos;
    
    % current manipulability and Jacobian
    mani =  robot1.model.maniplty(qCurrent);
    J = robot1.model.jacob0(qCurrent);
 
    % retrieve the current pose
    poseCurrent_Dobot = robot1.model.fkine(qCurrent);

    % current difference to the final position
    distanceDiff = transl(poseAboveVeggie_Dobot) - transl(poseCurrent_Dobot);
    angleDiff = tr2rpy(poseAboveVeggie_Dobot) - tr2rpy(poseCurrent_Dobot);

    % desired spatial velocity
    u = (distanceDiff/(steps-count))/timestep;
    omega = (angleDiff/(steps-count))/timestep;

    % reduce singularity if exists
    if mani < mani_threshold
        damping_coefficient = (1-(mani/mani_threshold)^2)*damping_coefficient_MAX;
        J_DLS = J'/(J*J'+ damping_coefficient*eye(6));    % damped least square Jacobian
        
        disp("DLS applied!");
        % joint velocity
        qd = J_DLS * [u; omega'];
    else
        % joint velocity
        qd = pinv(J) * [u; omega'];
    end

    % next position
    qNext = qCurrent + qd'*timestep;

    % check the validity of 'qNext':
    checkLimit = CheckJointLimit(robot1.model,qNext);
    if  checkLimit <= robot1.model.n
        disp(['Step ',num2str(count),'. Warning: exceed joint limit at joint ', num2str(checkLimit), ...
            '. A patch using IK solution is applied!']);

        % replace the invalid 'qNext' by an IK solution:
        poseNext = robot1.model.fkine(qNext);
        qNext = robot1.model.ikcon(poseNext,qCurrent);
    end

    % move robot
    robot1.MoveRobot(qNext);

    % retrieve the current pose
    poseCurrent_Dobot = robot1.model.fkine(robot1.model.getpos);
    pointCurrent_Dobot = poseCurrent_Dobot(1:3,4);
    error_displacement_dobot = norm(pointFinal_Dobot - pointCurrent_Dobot);

    count = count+1;
end


%% Lower the arm to pick the object 

% Gripped object's transform seen by the Dobot's end-effector
objectTr_Dobot = transl(0,0,-0.040);

if error_displacement_dobot <= 0.003
    posePick = transl(0,0,-0.04)*poseAboveVeggie_Dobot;
    
    robot1.MoveGripper(25);
    RMRCMotion(robot1,posePick,20,speedMax_Dobot);
    robot1.MoveGripper(24);

    % Bring the veggie to the chopping position

    % lift it up
    RMRCMotion(robot1,poseAboveVeggie_Dobot,20,speedMax_Dobot,veggie,objectTr_Dobot);

    % rotate the waist to the cutting board
    RMRCMotion(robot1,transl(0,0,0.04)*pose_Default_Dobot,50,speedMax_Dobot,veggie,objectTr_Dobot);

    % lower it down
    RMRCMotion(robot1,pose_Default_Dobot,20,speedMax_Dobot,veggie,objectTr_Dobot);
end

%% Replace the veggie with veggie pieces

veggiePose_current = veggie.pose;
delete(veggie.mesh_h);
delete(veggie);
hold on

veggiePiece = Veggie.empty(6,0);
for i = 1:6
    veggiePiece(i) = Veggie([veggie_Type,'Piece_',num2str(i),'.ply'],veggiePose_current,veggie_Color);
end

%% Dorna 2 joins
disp("Dorna starts!");

% Gripped object's transform seen by the Dorna's end-effector
objectTr_Dorna = transl(0.228,0,0.155)*rpy2tr(-2*pi,-pi/2,2*pi);

pose_First_Dorna = nan(4,4,5);
pose_Second_Dorna = nan(4,4,5);

q_First_Dorna =   [0,-61,82,-20,0] * pi/180;

% top right corner
% pose_Default_Dorna = robot2.model.fkine(robot2.jointDefault);

% top left corner
pose_First_Dorna(:,:,1) = robot2.model.fkine(q_First_Dorna);

% bottom left corner
pose_Second_Dorna(:,:,1) = transl([0 0 -0.04]) * pose_First_Dorna(:,:,1);

% bottom right corner
pose_Third_Dorna = transl([0 0 -0.05]) * pose_Default_Dorna;

% thickness of each slice
slice = 0.017;

pause()
for i = 1:4
    pose_First_Dorna(:,:,i+1) = transl(-slice,0,0)*pose_First_Dorna(:,:,i);
    pose_Second_Dorna(:,:,i+1) = transl(-slice,0,0)*pose_Second_Dorna(:,:,i);
end

% cutting operation
for i =1:5
    % reach the top left
    RMRCMotion(robot2,pose_First_Dorna(:,:,i),40,speedMax_Dorna);

    % chop down (bottom left)
    RMRCMotion(robot2,pose_Second_Dorna(:,:,i),40,speedMax_Dorna);

    % adjust the transform of each veggie piece
    adjustTr = transl(0,0,-(slice+0.005)*(i-1))*objectTr_Dorna;
    % push the veggie piece towards the bowl (bottom right)
    RMRCMotion(robot2,pose_Third_Dorna,40,speedMax_Dorna,veggiePiece(i),adjustTr);
    
    % veggie piece falls inside the bow
    RandominBowl(veggiePiece(i));
    
    % go back initial position(top right)
    RMRCMotion(robot2,pose_Default_Dorna,40,speedMax_Dorna);
end

%% Dobot bring the final piece closer

% lift it up
RMRCMotion(robot1,transl(0,0,0.04)*pose_Default_Dobot,20,speedMax_Dobot,veggiePiece(6),objectTr_Dobot);

% place it down
RMRCMotion(robot1,transl(0.1,0,0)*pose_Default_Dobot,20,speedMax_Dobot,veggiePiece(6),objectTr_Dobot);
robot1.MoveGripper(25);

% back to the default position
RMRCMotion(robot1,pose_Default_Dobot,40,speedMax_Dobot);
robot1.MoveGripper(0);

%% Dorna push the final piece down

% how the Dorna see the final piece
adjustTr_final = transl(0,0,-(slice+0.006)*6)*objectTr_Dorna;

% top left 
RMRCMotion(robot2,transl(-0.1,0,0)*pose_Default_Dorna,50,speedMax_Dorna);

% bottom left
RMRCMotion(robot2,transl(-0.1,0,-0.06)*pose_Default_Dorna,50,speedMax_Dorna);

% bottom right
RMRCMotion(robot2,pose_Third_Dorna,50,speedMax_Dorna,veggiePiece(6),adjustTr_final);

RandominBowl(veggiePiece(6));

% back to default position
RMRCMotion(robot2,pose_Default_Dorna,100,speedMax_Dorna);



