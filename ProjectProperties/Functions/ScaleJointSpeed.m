function [qdReturn,scale] = ScaleJointSpeed(qd,speedMax)
    scale = false;
    qdError = qd(find(qd>speedMax,size(qd,2)));
    
    if ~isempty(qdError)
        scale = speedMax/max(qdError);
        qdReturn = qd/scale;
    else
        qdReturn = qd;
    end
end
