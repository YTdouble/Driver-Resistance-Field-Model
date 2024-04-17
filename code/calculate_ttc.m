% Custom function to calculate the ttc time.
% The input a is the x-coordinate of car B in the new coordinate system, b is the y-coordinate, vA is the speed of car A, vB is the speed of car B, alpha is the direction of car B's speed.
% The output is the TTC time of the two cars.
function [ttcA,ttcB]=calculate_ttc(a,b,vA,vB,yawB)
    if vA < 0 || vB < 0
        error('The magnitude of velocity cannot be negative.');
    end
    if (b > 0 && yawB > 0) || (b < 0 && yawB < 0) || (yawB == 0)
        ttcB = inf;
        ttcA = -inf;
    else
        k = tan(yawB);
        y = 0;
        x = (y-b)./k + a;
        if x <= 0
            ttcB = inf;
            ttcA = -inf;
        else
            ttcA = x./vA;
            ttcB = sqrt((a-x).*(a-x) + b.*b) ./ vB;
        end
        
    end
end