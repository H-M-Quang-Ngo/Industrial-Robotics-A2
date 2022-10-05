classdef DobotMagicianwithGripper < handle
    %% DobotMagician
    properties(Access =public)
        %> Robot model
        model

        %> Gripper model
        modelGripper

        %> Transforms to adjust the position of gripper
        gripperTr = troty(pi/2)*trotx(pi);

        %> Robot name
        name = 'DobotMagicianwithGripper';

        %> Workspace
        workspace = [-2 2 -2 2 -0.55 2];

        %> Options to display robot
        plotopts = {'fps',240,'noarrow','nowrist','noname',...
            'tile1color',[1 1 1],'floorlevel',-0.55};

        %> Default set of joints
        defaultJoint  = [0,30,60,-45,0]*pi/180;

        %> Gripped object's transform (no orientation) seen by the end-effector
        objectTr = transl(0,0,-0.040);

    end

    methods (Access = public)
        %% Define robot Function
        function self = DobotMagicianwithGripper()
            self.CreateModel();
            self.PlotAndColourRobot();

            self.CreateGripperModel();
            self.PlotandColourGripper();
            
            % set initial robot position and gripper
            self.MoveRobot(self.defaultJoint);
            self.MoveGripper(0);
        end

        %% Create the robot model
        function CreateModel(self)
            L(1) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0,     'qlim',[deg2rad(-120),deg2rad(120)]);
            L(2) = Link('d',0,               'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(-5),deg2rad(90)]);
            L(3) = Link('d',0,               'a',0.147,  'alpha',0,      'offset',pi/4,  'qlim',[deg2rad(-15),deg2rad(90)]);
            L(4) = Link('d',0,               'a',0.06,   'alpha',pi/2,   'offset',0,     'qlim',[deg2rad(-140),deg2rad(140)]);
            L(5) = Link('d',-0.05,           'a',0,      'alpha',0,      'offset',pi,    'qlim',[deg2rad(-90),deg2rad(90)]);

            self.model = SerialLink(L,'name',self.name);
        end

        %% Plot and Colour Robot
        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.name,'_J',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n), self.plotopts{:}, 'workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                vertexColours = [0.5,0.5,0.5]; % Default if no colours in plyData

                if linkIndex == 5 % Link 5 is modified outside and have different struct format
                    try
                        vertexColours = [plyData{linkIndex+1}.vertex.x ...
                            , plyData{linkIndex+1}.vertex.y ...
                            , plyData{linkIndex+1}.vertex.z]/255;

                    catch ME_1
                        disp(ME_1);
                        disp('No vertex colours in plyData');
                        try
                            vertexColours = [plyData{linkIndex+1}.face.x ...
                                , plyData{linkIndex+1}.face.y ...
                                , plyData{linkIndex+1}.face.z]/255;
                        catch ME_1
                            disp(ME_1);
                            disp('Also, no face colours in plyData, so using a default colour');
                        end
                    end

                else
                    try
                        vertexColours = [plyData{linkIndex+1}.vertex.red ...
                            , plyData{linkIndex+1}.vertex.green ...
                            , plyData{linkIndex+1}.vertex.blue]/255;

                    catch ME_1
                        disp(ME_1);
                        disp('No vertex colours in plyData');
                        try
                            vertexColours = [plyData{linkIndex+1}.face.red ...
                                , plyData{linkIndex+1}.face.green ...
                                , plyData{linkIndex+1}.face.blue]/255;
                        catch ME_1
                            disp(ME_1);
                            disp('Also, no face colours in plyData, so using a default colour');
                        end
                    end
                end
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
        end

        %% Create the gripper model
        function CreateGripperModel(self)
            % Finger 1 of Gripper
            finger1 = Link('d',0,'a',0.06,'alpha',0,'offset',0, 'qlim',[deg2rad(0),deg2rad(25)]);
            self.modelGripper{1} = SerialLink(finger1,'name','Dobot Gripper Finger 1');

            % Finger 2 of Gripper
            finger2 = Link('d',0,'a',0.06,'alpha',0,'offset',0, 'qlim',[deg2rad(-25),deg2rad(0)]);
            self.modelGripper{2} = SerialLink(finger2,'name','Dobot Gripper Finger 2');
        end

        %% Plot and Colour Gripper
        function PlotandColourGripper(self)
            % Gripper Finger 1:
            for linkIndex = 0:self.modelGripper{1}.n
                [faceData, vertexData, plyData1{linkIndex+1}] = plyread(['DobotGripperFinger1_J',num2str(linkIndex),'.ply'],'tri');
                self.modelGripper{1}.faces{linkIndex+1} = faceData;
                self.modelGripper{1}.points{linkIndex+1} = vertexData;
            end

            % Gripper Finger 2:
            for linkIndex = 0:self.modelGripper{2}.n
                [faceData, vertexData, plyData2{linkIndex+1}] = plyread(['DobotGripperFinger2_J',num2str(linkIndex),'.ply'],'tri');
                self.modelGripper{2}.faces{linkIndex+1} = faceData;
                self.modelGripper{2}.points{linkIndex+1} = vertexData;
            end

            % Gripper's base on robot's end-effector
            self.modelGripper{1}.base = self.model.fkine(self.model.getpos)*self.gripperTr;
            self.modelGripper{2}.base = self.model.fkine(self.model.getpos)*self.gripperTr;

            % Display Gripper
            self.modelGripper{1}.plot3d(0, self.plotopts{:},'workspace',self.workspace);
            self.modelGripper{2}.plot3d(0, self.plotopts{:},'workspace',self.workspace);

            % Try to manually colour the Gripper
            for linkIndex = 0:self.modelGripper{1}.n
                handles1 = findobj('Tag', self.modelGripper{1}.name);
                h1 = get(handles1,'UserData');
                vertexColours = [0,0,0]; % Default colours

                handles2 = findobj('Tag', self.modelGripper{2}.name);
                h2 = get(handles2,'UserData');

                h1.link(linkIndex+1).Children.FaceVertexCData = repmat(vertexColours,[numrows(plyData1{linkIndex+1}.vertex.x) 1]);
                h1.link(linkIndex+1).Children.FaceColor = 'interp';

                h2.link(linkIndex+1).Children.FaceVertexCData = repmat(vertexColours,[numrows(plyData2{linkIndex+1}.vertex.x) 1]);
                h2.link(linkIndex+1).Children.FaceColor = 'interp';
            end
        end

        %% Update the gripper position by the robot's end joint
        function UpdateGripper(self)
            self.modelGripper{1}.base = self.model.fkine(self.model.getpos)*self.gripperTr;
            self.modelGripper{2}.base = self.model.fkine(self.model.getpos)*self.gripperTr;

            self.modelGripper{1}.animate(self.modelGripper{1}.getpos());
            self.modelGripper{2}.animate(self.modelGripper{2}.getpos());
        end

         %% Function to open/close gripper
        function MoveGripper(self,gripperAngle)
            % limit the input angle for gripper
            if gripperAngle > 25 || gripperAngle < 0
                gripperAngle = 12.5;
            end

            % current position (angle in rad) of the gripper
            pos1 = self.modelGripper{1}.getpos;
            pos2 = self.modelGripper{2}.getpos;

            steps = 20;
            % trajectory for finger 1
            q1 = lspb(pos1,gripperAngle*pi/180,steps);

            % trajectory for finger 2
            q2 = lspb(pos2,-gripperAngle*pi/180,steps);

            % move gripper's fingers by created trajectory
            for i =1:steps
                self.modelGripper{1}.animate(q1(i,:));
                self.modelGripper{2}.animate(q2(i,:));
                drawnow()
            end
        end

        %% Function to move the robot
        function MoveRobot(self,qTrajectory)
            if numrows(qTrajectory) == 1
                self.model.animate((qTrajectory));
                self.UpdateGripper();
            else
                for i=1:numrows(qTrajectory)
                    self.model.animate(qTrajectory(i,:));
                    self.UpdateGripper();
                    drawnow()
                end
            end
        end
  
        %% Test the movement of robot and gripper
        function TestMovement(self)
            qTest = rand(1,5);
            qTrajectory = jtraj(self.defaultJoint,qTest,50);

            for i = 1:50
                self.MoveRobot(qTrajectory(i,:));
                if mod(i,10) == 0
                    self.MoveGripper(15);
                    self.MoveGripper(0);
                end
            end
        end
    end
end
