% Comparison Model TTCP
%% Prejudgment, all models are the same
flag = 0;
if size(x1, 1) > size(x2, 1)
    flag = 1;
end

if flag == 0
    time_length = size(x1, 1); %获取矩阵行数, 若两车轨迹均持续4s，则行数与本车相同
else
    time_length = size(x2, 1); %若他车轨迹先消失，则仅计算他车轨迹存在时的SAB
end

%% Calculate the TTCP output
detaTTC = 0;
for i = 1:1:time_length-1
   %% Coordinate system transformation 
    % ground coordinate system to vehicle standard coordinate system
    Rotation_matrix = [cos(yawA(i)), sin(yawA(i)); -sin(yawA(i)), cos(yawA(i))];
    Translation_matrix = [x1(i); y1(i)];
    B_tmp = Rotation_matrix * ([x2(i); y2(i)] - Translation_matrix);
    x2_tmp = B_tmp(1); y2_tmp = B_tmp(2);
    v2_x = v2(i) .* cos(yawB(i)); v2_y = v2(i) .* sin(yawB(i));
    v_tmp = Rotation_matrix * [v2_x; v2_y];
    v2_tmp = norm([v_tmp(1)-v1(i); v_tmp(2)]);
    if norm(v_tmp) == 0
        yawB_tmp = 0;
    else
        yawB_tmp = atan2(v_tmp(2), v_tmp(1));
    end
    yawA_tmp = 0; a1_tmp = 0; v1_tmp = 0; x1_tmp = 0;y1_tmp = 0;
    
   %% Calculate the ttcp
   [ttcA, ttcB] = calculate_ttc(x2_tmp,y2_tmp,v1(i),v2(i),yawB_tmp);
   if abs(ttcA) ~= inf && abs(ttcB) ~= inf
       detattc = abs(ttcA - ttcB);
   else
       detattc = 120;
   end
   detaTTC(i) = detattc;
end

if flag == 0
    mean_ttc = mean(detaTTC);
    if mean_ttc > 120
        mean_ttc = 120;
    end
else
    mean_ttc = sum(detaTTC)/size(x1, 1);
    if mean_ttc > 120
        mean_ttc = 120;
    end
end