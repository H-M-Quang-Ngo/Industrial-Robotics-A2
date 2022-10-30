% clf;
% CreateEnvironment();
% arm = HumanArm;
% arm.model.base = transl(0.5,-1.3,0.2);
% arm.model.animate(0.25);
% 
gridSize = 50;
x = linspace(-0.15,0.8,gridSize)';
y1 = -0.72;
z = linspace(0.1,0.45,gridSize)';

gridLeft= nan(gridSize^2,3);

gridLeft(:,1) = repmat(x,gridSize,1);
gridLeft(:,2) = ones(gridSize^2,1)*y1;

for i = 1:gridSize
    gridLeft(1+gridSize*(i-1):gridSize*i,3) = z(i);
end

h = plot3(gridLeft(:,1),gridLeft(:,2),gridLeft(:,3));

% algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
% pointsInside = find(algebraicDist < 1);


