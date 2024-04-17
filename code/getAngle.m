% Calculate the angle between a point and the direction of vehicle speed.
% The inputs xc, yc are the coordinates of point C, and x2, y2, alpha are the coordinates of the vehicle and the direction of the velocity.
% The output angle is the angle between point C and the vehicle speed direction, range [0,pi], cosAngle is the cosine of the angle.
% If cosAngle is negative, it means that point C is behind the direction of speed of car B, which means the opposite direction.
function [angle, cosAngle] = getAngle(xc, yc, x2, y2, alpha)  
    OB = [cos(alpha), sin(alpha)];
    BC = [xc - x2, yc - y2];
    dotProduct = dot(OB, BC);
    productOfLengths = norm(OB) * norm(BC);
    cosAngle = dotProduct / productOfLengths;
    angle = acos(cosAngle);
end