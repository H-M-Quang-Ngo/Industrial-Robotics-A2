% Function to create and move veggie pieces
% to a random position inside the bow

function RandominBowl(veggiePiece)
    steps = 20;
    
    x = 0.08 + rand(100,1)*(0.5-0.45)-0.025;
    y = rand(100,1)*(0.06+0.06)-0.055; 
    z = 0.05 + rand(100,1)*0.02;
%   rpy = rand(1,3);
    
    T_rand = transl(x(50),y(50),-z(50));
    T = trinterp(veggiePiece.pose,T_rand*veggiePiece.pose,tpoly(0,1,steps));

    

    for j =1:steps
        veggiePiece.Update(T(:,:,j));
        drawnow();
    end
end