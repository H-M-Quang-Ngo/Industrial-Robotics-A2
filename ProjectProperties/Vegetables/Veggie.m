% and object can be gripped by the gripper
classdef Veggie < handle
    properties
        fileName
        pose
        vertexCount
        verts
        mesh_h
        colour
    end
    %%
    methods
        % place the object at pos = [x y] position and specify the color [r g b]
        function self = Veggie(fileName,pose,colour)
            % get the ply file's name
            self.fileName = fileName;
            
            if (nargin == 1)
                % default pose 
                self.pose = transl(0,0,0.02); 

                % default color
                self.colour = [255,128,0]/255; 

            elseif (nargin == 2)
                self.pose = pose;
                self.colour = [255,128,0]/255;

            else
                self.pose = pose;
                self.colour = colour;
            end
            
            self.PlotVeg();
        end

        % update the current position of the object. by pos = [x y]
        function Update(self, T) % T is an input object.Pose
            self.pose = T;
            updatedPoints = (T * [self.verts,ones(self.vertexCount,1)]')';
            self.mesh_h.Vertices = updatedPoints(:,1:3);
            drawnow()
        end

        % read the ply file and plot the model
        function PlotVeg(self)

            gcf;
            %these code below are modified from Gavin's
            %'PuttingSimulatedObjectsIntoTheEnvironment.m

            %Gavin (2022). Putting Simulated Objects Into The Environment
            %(https://www.mathworks.com/matlabcentral/fileexchange/58774-putting-simulated-objects-into-the-environment), MATLAB Central File Exchange. Retrieved August 13, 2022.

            [f,v,~] = plyread(self.fileName,'tri');

            % Get vertex count
            self.vertexCount = size(v,1);

            % Move center point to origin
            % midPoint = sum(v)/self.veg_VertexCount;
            % self.veg_Verts = v - repmat(midPoint,self.veg_VertexCount,1);
            self.verts = v;

            % get the colour
            vertexColours = ones(self.vertexCount,3).*self.colour;

            updatedPoints = (self.pose * [self.verts,ones(self.vertexCount,1)]')';

            self.mesh_h = trisurf(f,updatedPoints(:,1),updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end

        end
    end
end