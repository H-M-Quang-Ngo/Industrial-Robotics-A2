%% Create DH parameters for one-finger Gripper
% Finger 1 of Gripper
finger1 = Link('d',0,'a',0.06,'alpha',0,'offset',0, 'qlim',[deg2rad(0),deg2rad(25)]);
modelFinger{1} = SerialLink(finger1,'name','Dobot Gripper Finger 1');

% Finger 2 of Gripper
finger2 = Link('d',0,'a',0.06,'alpha',0,'offset',0, 'qlim',[deg2rad(-25),deg2rad(0)]);
modelFinger{2} = SerialLink(finger2,'name','Dobot Gripper Finger 2');

%% Create Gripper's Graphical Model
% Gripper Finger 1
for linkIndex = 0:modelFinger{1}.n
    [faceData, vertexData, plyData1{linkIndex+1}] = plyread(['DobotGripperFinger1_J',num2str(linkIndex),'.ply'],'tri');
    modelFinger{1}.faces{linkIndex+1} = faceData;
    modelFinger{1}.points{linkIndex+1} = vertexData;
end

% Gripper Finger 2:
for linkIndex = 0:modelFinger{2}.n
    [faceData, vertexData, plyData2{linkIndex+1}] = plyread(['DobotGripperFinger2_J',num2str(linkIndex),'.ply'],'tri');
    modelFinger{2}.faces{linkIndex+1} = faceData;
    modelFinger{2}.points{linkIndex+1} = vertexData;
end

% Display Gripper
modelFinger{1}.plot3d(0,'floorlevel',-1);
modelFinger{2}.plot3d(0,'floorlevel',-1);
zlim([-0.3 0.2]);

% Try to manually colour the Gripper
for linkIndex = 0:modelFinger{1}.n
    handles1 = findobj('Tag', modelFinger{1}.name);
    h1 = get(handles1,'UserData');
    vertexColours = [0,0.1,0]; % Default colours
    
    handles2 = findobj('Tag', modelFinger{2}.name);
    h2 = get(handles2,'UserData');

    h1.link(linkIndex+1).Children.FaceVertexCData = repmat(vertexColours,[numrows(plyData1{linkIndex+1}.vertex.x) 1]);
    h1.link(linkIndex+1).Children.FaceColor = 'interp';

    h2.link(linkIndex+1).Children.FaceVertexCData = repmat(vertexColours,[numrows(plyData2{linkIndex+1}.vertex.x) 1]);
    h2.link(linkIndex+1).Children.FaceColor = 'interp';
end
