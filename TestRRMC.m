clear
close all

r = DobotMagicianwithGripper;
hold on
h = PlaceObject('Carrot.ply',[0,-0.32,0.02]);

% desired joint state at final position
qPick = [-90 60 40 -10 90]*pi/180; 

T1 = r.model.fkine(r.model.getpos);
T2 = r.model.fkine(qPick);

% create a simple straight line trajectory
steps = 50;
T = trinterp(T1,T2,linspace(0,1,steps));

% safe step of time for not exceed each joint's max speed
timestep = 0.05;

% track the trajectory by RRMC
for i = 1:steps-1
    qCurrent = r.model.getpos;

    % spatial velocity
    u = (transl(T(:,:,i+1)) - transl(T(:,:,i)))/timestep;
    omega = (tr2rpy(T(:,:,i+1)) - tr2rpy(T(:,:,i+1)))/timestep;

    % joint velocity
    qd = pinv(r.model.jacob0(qCurrent)) * [u; omega'];

    % Next position
    qNext = qCurrent + qd'*timestep;

    % Check the validity of the next joint state:
    checkLimit = CheckJointLimit(r.model,qNext);
    if  checkLimit <= r.model.n
        disp(['Step ',num2str(i),'. Warning: exceed joint limit at joint ', num2str(checkLimit), ...
            '. A patch using IK solution is applied!']);

        % replace the invalid 'qNext' by an IK solution:
        poseNext = r.model.fkine(qNext);
        qNext = r.model.ikcon(poseNext,r.model.getpos);
    end

    r.MoveRobot(qNext);
    
    % try to correct the final position:
    if i == steps -1
        currentPose = r.model.fkine(r.model.getpos);
        error_displacement = norm(T2(1:3,4) - currentPose(1:3,4));
        
        % correct the position by jtraj if the error  > 5mm
        if error_displacement > 0.005
            qCorrect = jtraj(r.model.getpos,qPick,steps);
            r.MoveRobot(qCorrect);
        end
    end
end




