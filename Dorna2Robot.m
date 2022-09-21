classdef Dorna2Robot < handle
    properties
        model
        useGripper = false;
        workspace = [-2 2 -2 2 -0.3 2];
        plyFileNameStem = 'Dorna2Robot'
        defaultJoint = [0 -80 100 -20 0]*pi/180;
    end
    %% Define Robot Function
    methods
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

            self.model = SerialLink(L,'name','Dorna 2');
        end

        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n
                if self.useGripper && linkIndex == self.model.n
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'_J',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'_J',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
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
    end
end
