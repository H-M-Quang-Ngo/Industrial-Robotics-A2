% Function to create and move veggie pieces
% to a random position inside the bow

function T = RandominBowl(poseCurrent,steps)
    x = 0.08 + rand(100,1)*(0.5-0.45)-0.025;
    y = rand(100,1)*(0.06+0.06)-0.06; 
    z = 0.05 + rand(100,1)*0.02;
%     rpy = rand(1,3);

    T_rand = transl(x(50),y(50),-z(50));
    T = trinterp(poseCurrent,T_rand*poseCurrent,linspace(0,1,steps));
end