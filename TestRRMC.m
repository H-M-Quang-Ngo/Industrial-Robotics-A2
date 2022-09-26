clear
close all

r = DobotMagicianwithGripper;
hold on
h = PlaceObject('Carrot.ply',[0,-0.31,0.02]);

qPick = [-90 60 40 -10 0]*pi/180;
T1 = r.model.fkine(r.model.getpos);
T2 = r.model.fkine(qPick);

% create a simple straight line trajectory
steps = 50;
T = trinterp(T1,T2,linspace(0,1,steps));

timestep = 0.01;
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
    r.MoveRobot(qNext);
end




