function [qd_return,scale] = ScaleJointSpeed(qd,speedMax)
    scale = false;
    qd_error = qd(find(qd>speedMax,size(qd,2)));
    
    if ~isempty(qd_error)
        scale = speedMax/max(qd_error);
        qd_return = qd/scale;
    else
        qd_return = qd;
    end
end
