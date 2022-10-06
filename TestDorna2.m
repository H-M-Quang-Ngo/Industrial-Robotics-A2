robot = Dorna2Robot;
robot.model.base = transl([0.77 0 0]) * trotz(pi);
robot.MoveRobot(robot.model.getpos);

pose_First = nan(4,4,5);
pose_Second = nan(4,4,5);

q_First =   [0   -61    82   -20         0] * pi/180;
defaultJoint = [0 -78 102 -25 0]*pi/180;
pose_Default = robot.model.fkine(defaultJoint);
robot.MoveRobot(defaultJoint);

pose_First(:,:,1) = robot.model.fkine(q_First);

pose_Second(:,:,1) = transl([0 0 -0.04]) * pose_First(:,:,1);

pose_Third = transl([0 0 -0.05]) * pose_Default;


slice = 0.017;

for i = 1:4
    pose_First(:,:,i+1) = transl(-slice,0,0)*pose_First(:,:,i);
    pose_Second(:,:,i+1) = transl(-slice,0,0)*pose_Second(:,:,i);
end

% cutting operation
for i =1:5
    RMRCMotion(robot,pose_First(:,:,i),50);
    RMRCMotion(robot,pose_Second(:,:,i),50);
    RMRCMotion(robot,pose_Third,50,carrotPiece(i));
    RMRCMotion(robot,pose_Default,50);
end
