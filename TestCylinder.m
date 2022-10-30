% Create a cylinder point cloud
steps = 30;

r1 = linspace(0.3,0.01,steps);
pointCloudDobot = nan(steps^3,3);

for i = 1:steps
    [x1,y1,~] = cylinder(r1(i),steps - 1);
    x1 = x1(1,:)';
    y1 = y1(1,:)';

    z1 = linspace(0,0.4,steps)';
    p = nan(steps^2,3);

    for j = 1:steps
        p((steps*(j-1)+1):steps*j,:) = [x1 y1 repmat(z1(j),steps,1)];
    end

    pointCloudDobot(steps^2*(i-1)+1:steps^2*i,:) = p;
end