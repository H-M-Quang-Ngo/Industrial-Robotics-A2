clear 
clf
%% load robots
robot1 = DobotMagicianwithGripper;
robot2 = Dorna2Robot;
robot2.model.base = transl([0.77 0 0]) * trotz(pi);
robot2.MoveRobot(robot2.model.getpos);

%% set points
% Create image target (points in the image plane) 
pStar = [631 400 400 631; 400 400 631 631];

%Create four 3D points on Dorna 2 Robot
P=[0.4,0.4,0.4,0.4;
-0.025,0.025,0.025,-0.025;
 0.25,0.25,0.2,0.2];

%% set up camera for Dobot
% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'DobotCamera');

% frame rate
fps = 30;

% Define values
% gain of the controler
lambda = 0.8;

%% Display
xlim([-0.29 1.05])
ylim([-0.95 0.83])
zlim([-0.22 1.34])
view([227.15 15.28])

% plot camera and points
q0 = deg2rad([0 13 50 -20 0])';
robot1.MoveRobot(q0');

Tc0 = robot1.model.fkine(q0)*troty(-pi/2)*trotz(pi/2);
cam.T = Tc0;

% Display points in 3D and the camera
% cam.plot_camera('Tcam',Tc0,'scale',0.05);
h = patch(P(1,:),P(2,:),P(3,:),'b');
lighting gouraud
light


%% Trajectory for the Dorna
T1 = robot2.model.fkine(robot2.jointDefault);
T_Dorna = ctraj(T1,transl(0.07,0,0)*T1,200);
q_Dorna = robot2.model.ikcon(T_Dorna,robot2.jointDefault);

% Dorna and its points move
for i = 1:numrows(q_Dorna)
    robot2.MoveRobot(q_Dorna(i,:))
    P = homtrans(transl(0.07/200,0,0),P);
    h.Vertices =  homtrans(transl(0.07/200,0,0),h.Vertices')';
end

%% Project points to the image
cam.clf()
p = cam.plot(P, 'Tcam', Tc0);
cam.hold(true);
cam.plot(pStar, '*'); % create the camera view
pause()

%% Loop of the visual servoing
eMax = 100;

while eMax > 15
%         ksteps = ksteps + 1;

        % compute the view of the camera
        uv = cam.plot(P);
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        
        % compute the Jacobian
        % exact depth from simulation (not possible in practice)
        pt = homtrans(inv(cam.T), P);
        J = cam.visjac_p(uv, pt(3,:));
       
        % compute the velocity of camera in camera frame
        try
            vc = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', vc);

        % convert velocity of camera in camera frame to world frame
        J12 = [t2r(cam.T) zeros(3); zeros(3) t2r(cam.T)];
        v = J12*vc;
        
        % compute robot's Jacobian and inverse
        J2 = robot1.model.jacob0(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

        % Maximum angular velocity cannot exceed 320 degrees/s
        ind=find(qp>deg2rad(320));
        if ~isempty(ind)
             qp(ind)=deg2rad(320);
        end
        ind=find(qp<-deg2rad(320));
        if ~isempty(ind)
             qp(ind)=-deg2rad(320);
        end

        % Update joints 
        q = q0 + (1/fps)*qp;
        robot1.MoveRobot(q');

        % Get camera location
        Tc = robot1.model.fkine(q)*troty(-pi/2)*trotz(pi/2);
        cam.T = Tc;

%         if ~isempty(200) && (ksteps > 200)
%             break;
%         end
%         
        % update current joint position
        q0 = q;

        % update current error
        eMax = max(abs(e));
 end 

