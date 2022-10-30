clear
% close all

% set path
addpath(genpath('./ProjectProperties'));

%%
% create the environment
% CreateEnvironment();

% type of the veggie to cut
veggieType = 'Cucumber';

switch veggieType
    case 'Carrot'
        veggieColor = [255,128,0]/255;

        % initial position above the veggie
        veggiePose = transl(0,-0.276,0.02);

        % desired joint state for the dobot at the upper picking position
        qPickDobot = [-90 45 35 -35 -90]*pi/180;

    case 'Cucumber'
        veggieColor = [30,170,30]/255;
        veggiePose = transl(0.01,-0.23,0.02);
        qPickDobot = [-88.86 30.197 60.7302 -45.93 -90]*pi/180;
end

veggie = Veggie([veggieType,'.ply'],veggiePose,veggieColor);

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
poseDefaultDobot = robot1.model.fkine(robot1.jointDefault);
poseDefaultDorna = robot2.model.fkine(robot2.jointDefault);

% required initial and the pose above the veggie
poseCurrentDobot = robot1.model.fkine(robot1.model.getpos);
poseAboveVeggieDobot = robot1.model.fkine(qPickDobot);

pointCurrentDobot = poseCurrentDobot(1:3,4);
pointFinalDobot = poseAboveVeggieDobot(1:3,4);
errorDisplacementDobot = norm(pointFinalDobot - pointCurrentDobot);

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
checkCollision =60;

count = 0;
pause();

disp("Dobot starts!");

%% Dobot tracks the trajectory by RRMC to reach the veggie
% the closer the obstacle to the target, the less accurate of the
% avoidance

while errorDisplacementDobot > 0.005

    % trace the end-effector
    % plot3(poseCurrent(1,4),poseCurrent(2,4),poseCurrent(3,4),'r.');

    % check collision and try to avoid
    if count == checkCollision
        disp(['Collision detected at step ',num2str(count)]);

        % try to avoid that collision if possible
        if (count/steps) < 3/5
            % lift the arm
            zLift = 0.2;
            T_Lift = transl(0,0,zLift)*poseCurrentDobot;
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
    poseCurrentDobot = robot1.model.fkine(qCurrent);

    % current difference to the final position
    distanceDiff = transl(poseAboveVeggieDobot) - transl(poseCurrentDobot);
    angleDiff = tr2rpy(poseAboveVeggieDobot) - tr2rpy(poseCurrentDobot);

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
    poseCurrentDobot = robot1.model.fkine(robot1.model.getpos);
    pointCurrentDobot = poseCurrentDobot(1:3,4);
    errorDisplacementDobot = norm(pointFinalDobot - pointCurrentDobot);

    count = count+1;
end


%% Lower the arm to pick the object 

% Gripped object's transform seen by the Dobot's end-effector
objectTrDobot = transl(0,0,-0.040);

if errorDisplacementDobot <= 0.005
    posePick = transl(0,0,-0.04)*poseAboveVeggieDobot;
    
    robot1.MoveGripper(25);
    RMRCMotion(robot1,posePick,20,speedMax_Dobot);
    robot1.MoveGripper(24);

    % Bring the veggie to the chopping position

    % lift it up
    RMRCMotion(robot1,poseAboveVeggieDobot,20,speedMax_Dobot,veggie,objectTrDobot);

    % rotate the waist to the cutting board
    RMRCMotion(robot1,transl(0,0,0.04)*poseDefaultDobot,50,speedMax_Dobot,veggie,objectTrDobot);

    % lower it down
    RMRCMotion(robot1,poseDefaultDobot,20,speedMax_Dobot,veggie,objectTrDobot);
end

%% Replace the veggie with veggie pieces

veggiePoseCurrent = veggie.pose;
delete(veggie.mesh_h);
delete(veggie);
hold on

veggiePiece = Veggie.empty(6,0);
for i = 1:6
    veggiePiece(i) = Veggie([veggieType,'Piece_',num2str(i),'.ply'],veggiePoseCurrent,veggieColor);
end

%% Dorna 2 joins
disp("Dorna starts!");

% Gripped object's transform seen by the Dorna's end-effector
objectTrDorna = transl(0.228,0,0.155)*rpy2tr(-2*pi,-pi/2,2*pi);

poseFirstDorna = nan(4,4,5);
poseSecondDorna = nan(4,4,5);

qFirstDorna =   [0,-61,82,-20,0] * pi/180;

% top right corner
% pose_Default_Dorna = robot2.model.fkine(robot2.jointDefault);

% top left corner
poseFirstDorna(:,:,1) = robot2.model.fkine(qFirstDorna);

% bottom left corner
poseSecondDorna(:,:,1) = transl([0 0 -0.04]) * poseFirstDorna(:,:,1);

% bottom right corner
poseThirdDorna = transl([0 0 -0.05]) * poseDefaultDorna;

% thickness of each slice
slice = 0.017;

pause()
for i = 1:4
    poseFirstDorna(:,:,i+1) = transl(-slice,0,0)*poseFirstDorna(:,:,i);
    poseSecondDorna(:,:,i+1) = transl(-slice,0,0)*poseSecondDorna(:,:,i);
end

% cutting operation
for i =1:5
    % reach the top left
    RMRCMotion(robot2,poseFirstDorna(:,:,i),40,speedMax_Dorna);

    % chop down (bottom left)
    RMRCMotion(robot2,poseSecondDorna(:,:,i),40,speedMax_Dorna);

    % adjust the transform of each veggie piece
    adjustTr = transl(0,0,-(slice+0.005)*(i-1))*objectTrDorna;
    % push the veggie piece towards the bowl (bottom right)
    RMRCMotion(robot2,poseThirdDorna,40,speedMax_Dorna,veggiePiece(i),adjustTr);
    
    % veggie piece falls inside the bow
    RandominBowl(veggiePiece(i));
    
    % go back initial position(top right)
    RMRCMotion(robot2,poseDefaultDorna,40,speedMax_Dorna);
end

%% Dobot bring the final piece closer

% lift it up
RMRCMotion(robot1,transl(0,0,0.04)*poseDefaultDobot,20,speedMax_Dobot,veggiePiece(6),objectTrDobot);

% place it down
RMRCMotion(robot1,transl(0.1,0,0)*poseDefaultDobot,20,speedMax_Dobot,veggiePiece(6),objectTrDobot);
robot1.MoveGripper(25);

% back to the default position
RMRCMotion(robot1,poseDefaultDobot,40,speedMax_Dobot);
robot1.MoveGripper(0);

%% Dorna push the final piece down

% how the Dorna see the final piece
adjustTrFinal = transl(0,0,-(slice+0.006)*6)*objectTrDorna;

% top left 
RMRCMotion(robot2,transl(-0.1,0,0)*poseDefaultDorna,50,speedMax_Dorna);

% bottom left
RMRCMotion(robot2,transl(-0.1,0,-0.06)*poseDefaultDorna,50,speedMax_Dorna);

% bottom right
RMRCMotion(robot2,poseThirdDorna,50,speedMax_Dorna,veggiePiece(6),adjustTrFinal);

RandominBowl(veggiePiece(6));

% back to default position
RMRCMotion(robot2,poseDefaultDorna,100,speedMax_Dorna);



