% Test the cutting operation of Dorna 2

% set path
addpath(genpath('./ProjectProperties'));

%%
robot = Dorna2Robot;
robot.model.base = transl([0.77 0 0]) * trotz(pi);
robot.MoveRobot(robot.model.getpos);

poseFirst = nan(4,4,5);
poseSecond = nan(4,4,5);

qFirst =   [0   -61    82   -20         0] * pi/180;
jointDefault = [0 -78 102 -25 0]*pi/180;
poseDefault = robot.model.fkine(jointDefault);
robot.MoveRobot(jointDefault);

poseFirst(:,:,1) = robot.model.fkine(qFirst);

poseSecond(:,:,1) = transl([0 0 -0.04]) * poseFirst(:,:,1);

poseThird = transl([0 0 -0.05]) * poseDefault;


slice = 0.017;

for i = 1:4
    poseFirst(:,:,i+1) = transl(-slice,0,0)*poseFirst(:,:,i);
    poseSecond(:,:,i+1) = transl(-slice,0,0)*poseSecond(:,:,i);
end

% cutting operation
for i =1:5
    RMRCMotion(robot,poseFirst(:,:,i),50);
    RMRCMotion(robot,poseSecond(:,:,i),50);
    RMRCMotion(robot,poseThird,50,carrotPiece(i));
    RMRCMotion(robot,poseDefault,50);
end
