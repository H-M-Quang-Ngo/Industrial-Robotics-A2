function check = CheckJointLimit(robot,q)
    qlim = robot.qlim;
    
    for i = 1:robot.n
        check  = i;
        if q(i) < qlim(i,1) || qlim(i,2) < q(i)
            return
        end
    end

    check = i+1;
end