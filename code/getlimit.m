% Calculate the intersection of the DRFM trapezoids of two cars.
% The input alpha is a 1*2 vector of the heading angles of car A and car B, respectively.
% The input x and y are 1*2 vectors of the coordinates of car A and car B at the current moment, respectively.
% The output is the set of coordinates of the intersection of the DRFMs of the two cars.
function [X, Y, t1, t2, t3] = getlimit(alpha,x,y)
    d = 12;
    flag_merge = 0;
    angle_limit = 0.05;
    % Calculate the four vertices of the DRFM of car A
    x1A = x(1)-1*sin(alpha(1));
    x2A = x(1)+1*sin(alpha(1));
    y1A = y(1)+1*cos(alpha(1));
    y2A = y(1)-1*cos(alpha(1));
    alpha1A = alpha(1)+pi/12;
    alpha2A = alpha(1)-pi/12;
    [x3A,y3A,x4A,y4A] = getbottom(x1A,y1A,x2A,y2A,alpha1A,alpha2A,d);
    t1 = polyshape([x1A x3A x4A x2A], [y1A y3A y4A y2A]); 
    % Calculate the four vertices of the DRFM of car B
    x1B = x(2)-1*sin(alpha(2));
    x2B = x(2)+1*sin(alpha(2));
    y1B = y(2)+1*cos(alpha(2));
    y2B = y(2)-1*cos(alpha(2));
    alpha1B = alpha(2)+pi/12;
    alpha2B = alpha(2)-pi/12;
    [x3B,y3B,x4B,y4B] = getbottom(x1B,y1B,x2B,y2B,alpha1B,alpha2B,d);
    t2 = polyshape([x1B x3B x4B x2B], [y1B y3B y4B y2B]);
    % Calculate the intersection of two trapezoids
    t3 = intersect(t1, t2);
    i = 1;
    if isempty(t3.Vertices)  % If there are no intersections
        X = 0;
        Y = 0;
    elseif inpolygon(x1A,y1A,[x1B x3B x4B x2B],[y1B y3B y4B y2B]) && inpolygon(x2A,y2A,[x1B x3B x4B x2B],[y1B y3B y4B y2B]) && (abs(alpha(1)-alpha(2))<pi/2) && flag_merge == 0
        % If the special operator omega2 is satisfied
        X = 0;
        Y = 0;
    elseif abs(alpha(1)-alpha(2))<angle_limit || abs(alpha(1)-alpha(2))>(pi-angle_limit)
        % If the special operator omega1 is satisfied
        X = 0;
        Y = 0;
    else
        xv = t3.Vertices(:,1);
        yv = t3.Vertices(:,2);
        max_size = ceil(((max(xv)-min(xv))/0.5+1)*((max(yv)-min(yv))/0.5+1));
        XX = zeros(1,max_size);
        YY = zeros(1,max_size);
        for px = min(xv):(d/25):max(xv)
            for py = min(yv):(d/25):max(yv)
                if inpolygon(px,py,xv,yv)
                    XX(i) = px;
                    YY(i) = py;
                    i = i+1;
                end      
            end
        end
        X = XX(1,1:i-1);
        Y = YY(1,1:i-1);       
    end
end

% Input the coordinates of the upper base and the waist length to calculate the coordinates of the lower base of the trapezoid
function [x3,y3,x4,y4] = getbottom(x1,y1,x2,y2,alpha1,alpha2,d)
    k1 = tan(alpha1);
    b1 = y1 - k1*x1;
    x3_1 = x1 + d/sqrt(1+k1^2);
    y3_1 = k1*x3_1 + b1;
    x3_2 = x1 - d/sqrt(1+k1^2);
    y3_2 = k1*x3_2 + b1;
    [~, cosa1] = getAngle(x3_1,y3_1,x1,y1,alpha1);
    [~, cosa2] = getAngle(x3_2,y3_2,x1,y1,alpha1);
    if cosa1 > 0
        x3 = x3_1;
        y3 = y3_1;
    elseif cosa2 > 0
        x3 = x3_2;
        y3 = y3_2;
    else
        error('ERROR! The DRFM endpoints are not in the direction of vehicle movement.')
    end
    k2 = tan(alpha2);
    b2 = y2 -k2.*x2;
    x4_1 = x2 + d/sqrt(1+k2^2);
    y4_1 = k2*x4_1 + b2;
    x4_2 = x2 - d/sqrt(1+k2^2);
    y4_2 = k2*x4_2 + b2;
    [~, cosa1] = getAngle(x4_1,y4_1,x2,y2,alpha2);
    [~, cosa2] = getAngle(x4_2,y4_2,x2,y2,alpha2);
    if cosa1 > 0
        x4 = x4_1;
        y4 = y4_1;
    elseif cosa2 > 0
        x4 = x4_2;
        y4 = y4_2;
    else
        error('ERROR! The DRFM endpoints are not in the direction of vehicle movement.')
    end    
end