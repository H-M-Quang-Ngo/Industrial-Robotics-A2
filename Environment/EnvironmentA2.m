close all
clear;
hold on
axis ([-1 6 -3 2.8 -1 3])
grid on
xlabel ('X');
ylabel ('Y');
zlabel ('Z');
axis equal;  
surf([-1,-1;6,6],[-3,2.8;-3,2.8],[-1, -1;-1,-1],'CData',imread('Floor.jpg'),'FaceColor','texturemap');
surf([-1,-1;6,6],[2.8,2.8;2.8,2.8],[-1,3;-1,3],'CData',imread('SideView.jpg'),'FaceColor','texturemap');
surf([2.5,2.5;2.9,2.9],[2.3,2.3;2.3,2.3],[0.7,1.3;0.7,1.3],'CData',imread('Sign2.png'),'FaceColor','texturemap');
surf([0.1,0.1;0.8,0.8],[2.3,2.3;2.3,2.3],[0.7,1.3;0.7,1.3],'CData',imread('Sign3.png'),'FaceColor','texturemap');
surf([-0.58,-0.58;-0.58,-0.58],[-1.5,-1.5;1.5,1.5],[0.7,1.7;0.7,1.7],'CData',imread('Design1.png'),'FaceColor','texturemap');
surf([-0.58,-0.58;-0.58,-0.58],[-2.7,-2.7;-1.8,-1.8],[0.5,1.5;0.5,1.5],'CData',imread('Sign4.png'),'FaceColor','texturemap');
surf([-0.58,-0.58;-0.58,-0.58],[-0.8,-0.8;0.8,0.8],[2,2.9;2,2.9],'CData',imread('Design2.png'),'FaceColor','texturemap');
surf([-0.58,-0.58;-0.58,-0.58],[-0.9,-0.9;0.6,0.6],[0,0.5;0,0.5],'CData',imread('Design3.png'),'FaceColor','texturemap');
PlaceObject("CuttingBoard.ply", [ 0.002 0 -0.56 ]);
PlaceObject("SupportPillar.ply", [0.3 0 -0.56]);
PlaceObject("Bowl1.ply", [ 0 0 -0.013])
PlaceObject("FoodTray.ply", [ 0 0 -0.55])
PlaceObject("Base.ply", [ 0 0 -0.55])
PlaceObject("Base2.ply", [ 0 0 -0.55])
PlaceObject("Model1.ply", [ 1 -1.4 -0.85])
PlaceObject("Wall.ply", [ -1 -0.5 2])
PlaceObject("FamilyTable.ply", [ 3.5 -2 -0.7])
PlaceObject("FireAndFirstAid.ply", [ 5.4 2.6 -1 ])
PlaceObject("CCTV.ply",[-0.4 2.5 3])
PlaceObject("CCTV2.ply", [ -0.4 -2.8 2.8])
PlaceObject("ModelKitchen.ply", [ 2.4 1.3 -1.105])
PlaceObject("MainWorker.ply", [ 0.3 1 -1.15])
PlaceObject("PowerSocket.ply", [ -0.58 -0.6 -0.7 ])
PlaceObject("PowerSocket.ply", [ -0.58 -2 -0.7 ])
PlaceObject("Desk.ply", [ -0.17 -2 -0.32])
PlaceObject("imac.ply", [ -0.35 -2 0])
PlaceObject("Keyboard.ply", [ -0.05 -2 -0.28])
PlaceObject("MagicMouse.ply", [ -0.1 -2.5 -0.3])
PlaceObject("Speaker.ply", [ -0.38 -2.5 -0.3])
PlaceObject("Speaker.ply", [ -0.38 -1.5 -0.3])
PlaceObject("Estop.ply", [ -0.25 -1.5 -0.3])
PlaceObject("Estop2.ply", [ -0.25 -1.4 -0.3])
PlaceObject("WarningLight.ply", [ 1.1 -0.7 0])
PlaceObject("Shiled.ply", [ 1.15 0.05 0.25 ])
PlaceObject("Controller1.ply", [ -0.1 -1.5 -0.25])
PlaceObject("Controller2.ply", [ 0.1 -1.5 -0.25])
camlight
lighting flat