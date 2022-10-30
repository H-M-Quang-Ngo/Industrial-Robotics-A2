classdef HumanArm <handle
    properties
        %> Arm model
        model

        %> Options to display
        plotopts = {'fps',240,'noarrow','nowrist','noname',...
            'tile1color',[1 1 1],'floorlevel',-1.5};

        %> Workspace
        workspace = [-1 2 -2 2 -1.5 2];

        %> Ellipsoid model default at position 0
        center = [0.6,0,0]

        radii = [0.6 0.13 0.13];
    end

    methods
        %% Define the function
        function self = HumanArm()
            self.CreateArm();
            self.MoveArm(0);
        end

        %% Construct DH and Plot
        function CreateArm(self)
            % create DH parameters
            L = Link('d',0, 'a',1.2,'alpha',0,'offset',0);
            self.model = SerialLink(L,'name','Human Arm');

            % plot and colour
            for linkIndex = 0:self.model.n
                [faceData, vertexData, plyData{linkIndex+1}] = plyread(['HumanArm_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            self.model.plot3d(0, self.plotopts{:}, 'workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;

            % try to manually colour the arm
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                vertexColours = [183 128 17]/255; % Default colours
                h.link(linkIndex+1).Children.FaceVertexCData = repmat(vertexColours,[numrows(plyData{linkIndex+1}.vertex.x) 1]);
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
        end

        %% Function to move the arm
        function MoveArm(self,q)
            for i = 1:numrows(q)
                self.model.animate(q(i));
                drawnow()
            end
        end
    end
end


