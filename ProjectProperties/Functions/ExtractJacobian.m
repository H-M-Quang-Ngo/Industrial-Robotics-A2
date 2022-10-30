function manipulability = ExtractJacobian(J,option)
    switch option
        case 'x'
            Jm = J(1,:);
        case 'y'
            Jm = J(2,:);
        case 'z'
            Jm = J(3,:);
        case 'rx'
            Jm = J(4,:);
        case 'ry'
            Jm = J(5,:);
        case 'rz'
            Jm = J(6,:);
        case 'T'
            Jm = J(1:3,:);
        case 'R'
            Jm = J(4:6,:);
    end
    
    manipulability = sqrt(det(Jm*Jm'));
    disp(['mani in ',option,': ',num2str(manipulability)]);
end