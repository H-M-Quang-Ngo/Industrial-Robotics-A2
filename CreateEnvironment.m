function CreateEnvironment(~)
    close all
    clear;
    hold on
    axis ([-1 6 -3 2.8 -1 3])
    grid on
    xlabel ('X');
    ylabel ('Y');
    zlabel ('Z');
    axis equal;  
    surf([-1,-1;6,6],[-3,2.8;-3,2.8],[-1, -1;-1,-1],'CData',imread('Env_Floor.jpg'),'FaceColor','texturemap');
    surf([-1,-1;6,6],[2.8,2.8;2.8,2.8],[-1,3;-1,3],'CData',imread('Env_SideView.jpg'),'FaceColor','texturemap');
    surf([2.5,2.5;2.9,2.9],[2.3,2.3;2.3,2.3],[0.7,1.3;0.7,1.3],'CData',imread('Env_Sign2.png'),'FaceColor','texturemap');
    surf([0.1,0.1;0.8,0.8],[2.3,2.3;2.3,2.3],[0.7,1.3;0.7,1.3],'CData',imread('Env_Sign3.png'),'FaceColor','texturemap');
    surf([-0.58,-0.58;-0.58,-0.58],[-1.5,-1.5;1.5,1.5],[0.7,1.7;0.7,1.7],'CData',imread('Env_Design1.png'),'FaceColor','texturemap');
    surf([-0.58,-0.58;-0.58,-0.58],[-2.7,-2.7;-1.8,-1.8],[0.5,1.5;0.5,1.5],'CData',imread('Env_Sign4.png'),'FaceColor','texturemap');
    surf([-0.58,-0.58;-0.58,-0.58],[-0.8,-0.8;0.8,0.8],[2,2.9;2,2.9],'CData',imread('Env_Design2.png'),'FaceColor','texturemap');
    surf([-0.58,-0.58;-0.58,-0.58],[-0.9,-0.9;0.6,0.6],[0,0.5;0,0.5],'CData',imread('Env_Design3.png'),'FaceColor','texturemap');
    
    PlaceObject("Env_CuttingBoard.ply", [ 0.002 0 -0.56 ]);
    PlaceObject("Env_SupportPillar.ply", [0.3 0 -0.56]);
    PlaceObject("Env_Bowl1.ply", [ 0 0 -0.013]);
    PlaceObject("Env_FoodTray.ply", [ -0.03 -0.045 -0.55]);
    PlaceObject("Env_Base.ply", [ 0 0 -0.55]);
    PlaceObject("Env_Base2.ply", [ 0.1 0 -0.55]);
    PlaceObject("Env_Model1.ply", [ 1 -1.4 -0.85]);
    PlaceObject("Env_Wall.ply", [ -1 -0.5 2]);
    PlaceObject("Env_FamilyTable.ply", [ 3.5 -2 -0.7]);
    PlaceObject("Env_FireAndFirstAid.ply", [ 5.4 2.6 -1 ]);
    PlaceObject("Env_CCTV.ply",[-0.4 2.5 3]);
    PlaceObject("Env_CCTV2.ply", [ -0.4 -2.8 2.8]);
    PlaceObject("Env_ModelKitchen.ply", [ 2.4 1.3 -1.105]);
    PlaceObject("Env_MainWorker.ply", [ 0.3 1 -1.15]);
    PlaceObject("Env_PowerSocket.ply", [ -0.58 -0.6 -0.7 ]);
    PlaceObject("Env_PowerSocket.ply", [ -0.58 -2 -0.7 ]);
    PlaceObject("Env_Desk.ply", [ -0.17 -2 -0.32]);
    PlaceObject("Env_imac.ply", [ -0.35 -2 0]);
    PlaceObject("Env_Keyboard.ply", [ -0.05 -2 -0.28]);
    PlaceObject("Env_MagicMouse.ply", [ -0.1 -2.5 -0.3]);
    PlaceObject("Env_Speaker.ply", [ -0.38 -2.5 -0.3]);
    PlaceObject("Env_Speaker.ply", [ -0.38 -1.5 -0.3]);
    PlaceObject("Env_Estop.ply", [ -0.25 -1.5 -0.3]);
    PlaceObject("Env_Estop2.ply", [ -0.25 -1.4 -0.3]);
    PlaceObject("Env_WarningLight.ply", [ 1.1 -0.7 0]);
    PlaceObject("Env_Shiled.ply", [ 1.15 0.05 0.25 ]);
    PlaceObject("Env_Controller1.ply", [ -0.1 -1.5 -0.25]);
    PlaceObject("Env_Controller2.ply", [ 0.1 -1.5 -0.25]);
    PlaceObject("Env_LightCurtain.ply", [ 0.1 0.1 -0.08]);
    PlaceObject("Env_LightCurtain.ply", [ 0.1 -1 -0.08]);
    
    carrotColor = [255,128,0]/255;
    
    Veggie('Carrot.ply',transl(0.03,-0.32,0.02),carrotColor);
    
    camlight
end
