classdef Dorna2Robot < handle
    properties
        %> Robot model
        model

        %> Robot name
        name = 'Dorna2Robot'

        %> Workspace
        workspace = [-2 2 -2 2 -0.3 2];

        %> Options to display robot
        plotopts = {'fps',240,'noarrow','nowrist','noname','tile1color',[1 1 1],'floorlevel',-0.3};
                
        %> default set of joints
        defaultJoint = [0 -78 102 -25 0]*pi/180;

        %> Gripped object's transform (no orientation) seen by the end-effector
        objectTr = transl(0.224,0,0.16)*rpy2tr(-2*pi,-pi/2,2*pi);
    end


    methods
        %% Define Robot Function
        function self = Dorna2Robot()
            self.CreateModel();
            self.PlotAndColourRobot();
            self.model.animate(self.defaultJoint)
        end

        %% Model Robot's DH Parameters
        function CreateModel(self)
            L(1) = Link('d',0.2064,   'a',0.09548, 'alpha',-pi/2, 'offset',0,     'qlim',[deg2rad(-160),deg2rad(180)]);
            L(2) = Link('d',0,        'a',0.2032,  'alpha',0,     'offset',0,     'qlim',[deg2rad(-180),deg2rad(90)]);
            L(3) = Link('d',0,        'a',0.1524,  'alpha',0,     'offset',0,     'qlim',[deg2rad(-142),deg2rad(142)]);
            L(4) = Link('d',0,        'a',0,       'alpha',pi/2,  'offset',pi/2,  'qlim',[deg2rad(-135),deg2rad(135)]);
            L(5) = Link('d',0.04892,  'a',0,       'alpha',0,     'offset',0,     'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink(L,'name',self.name);
        end

        %% Plot and Colour Robot
        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n
                [faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.name,'_J',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),self.plotopts{:},'workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;

            % Try to manually colour the arm
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');

                switch (linkIndex + 1)
                    case 1
                        vertexColours = [0,0.1,1];
                    case 5
                        vertexColours = [0,0,0.5];
                    case 6
                        vertexColours = [1, 1, 1];
                    otherwise
                        vertexColours = [0,0.4,1]; % Default colours
                end
                h.link(linkIndex+1).Children.FaceVertexCData = repmat(vertexColours,[numrows(plyData{linkIndex+1}.vertex.x) 1]);
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
        end

        %% Function to move the robot
        function MoveRobot(self,qTrajectory)
            for i=1:numrows(qTrajectory)
                self.model.animate(qTrajectory(i,:));
                drawnow()
            end
        end

        %% Test the movement of robot
        function TestMovement(self)
            qTest = rand(1,5);
            qTrajectory = jtraj(self.defaultJoint,qTest,100);
            self.MoveRobot(qTrajectory);
        end
    end
end


