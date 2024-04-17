% The code of the DRFM
%% Initialization parameters
flag = 0;
Field = 0;

%% Prejudge
if size(x1, 1) > size(x2, 1) % Judge whether the other car disappears from the scene before the ego car.
    flag = 1;
end

if flag == 0
    time_length = size(x1, 1);
else
    time_length = size(x2, 1);
end

%% Calculate relative distance
R2 = (x1(1:time_length)-x2(1:time_length)).*(x1(1:time_length)-x2(1:time_length))+(y1(1:time_length)-y2(1:time_length)).*(y1(1:time_length)-y2(1:time_length));

%% Calculate interaction strength
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
      
   %% Calculate DRFM
      % Obtain preview moment
      if (i+15) <= time_length
          pre_time = i+15;
      else
          pre_time = time_length;
      end
      
      if abs(v1(i)) < 1 && abs(v2(i)) < 1  % If both cars are stationary
          A_tmp_final = Rotation_matrix * ([x1(time_length); y1(time_length)] - Translation_matrix);
          x1_tmp_final = A_tmp_final(1); y1_tmp_final = A_tmp_final(2); 
          if y1_tmp_final==0 && x1_tmp_final==0
              yawA_final = yawA_tmp;
          else
              yawA_final = atan2(y1_tmp_final-0,x1_tmp_final-0); 
          end 
          B_tmp_final = Rotation_matrix * ([x2(time_length); y2(time_length)] - Translation_matrix);
          x2_tmp_final = B_tmp_final(1); y2_tmp_final = B_tmp_final(2); 
          if y2_tmp_final-y2_tmp==0 && x2_tmp_final-x2_tmp==0
              yawB_final = yawB_tmp;
          else
              yawB_final = atan2(y2_tmp_final-y2_tmp,x2_tmp_final-x2_tmp);
          end
          % Custom function to calculate the intersection of the DRFMs of the two cars
         [X,Y,t1,t2,t3] = getlimit([yawA_final,yawB_final],[0,x2_tmp],[0,y2_tmp]);
          
          % Calculate the sum of the intensities in the intersection region
          pv1 = 1; % Speed at standstill is treated as vehicle idle speed
          pv2 = 1;
          field1 = zeros(size(X,2),size(Y,2));
          field2 = zeros(size(X,2),size(Y,2));
          if all(X == 0) && all(Y == 0) && length(X)==1
             Field(i) = 0;
          else
             for j = 1:1:size(X,2)
                 for m = 1:1:size(Y,2)
                     pd1 = (X(j).*X(j)) + (Y(m).*Y(m));
                     pd2 = (X(j)-x2_tmp).*(X(j)-x2_tmp) + (Y(m)-y2_tmp).*(Y(m)-y2_tmp);
                     [angel1, cosangle1] = getAngle(X(j),Y(m),0,0,yawA_final); % Custom function to calculate the angle between a point and the direction of vehicle speed
                     field1(j,m) = pv1 ./ pd1 .* cosangle1 .* cosangle1;
                     [angel2, cosangle2] = getAngle(X(j),Y(m),x2_tmp,y2_tmp,yawB_final);
                     field2(j,m) = pv2 ./ pd2 .* cosangle2.* cosangle2;
                 end
             end
             Field(i) = sum(field1(~isnan(field1) & ~isinf(field1))) + sum(field2(~isnan(field2) & ~isinf(field2)));
          end
          
      elseif abs(v1(i)) < 1   % If only the ego car is stationary
         A_tmp_final = Rotation_matrix * ([x1(time_length); y1(time_length)] - Translation_matrix);
         x1_tmp_final = A_tmp_final(1); y1_tmp_final = A_tmp_final(2); 
         if y1_tmp_final==0 && x1_tmp_final==0
             yawA_final = yawA_tmp;
         else
             yawA_final = atan2(y1_tmp_final-0,x1_tmp_final-0); 
         end
         B_tmp_pre = Rotation_matrix * ([x2(pre_time); y2(pre_time)] - Translation_matrix);
         x2_tmp_pre = B_tmp_pre(1); y2_tmp_pre = B_tmp_pre(2);
         if y2_tmp_pre-y2_tmp==0 && x2_tmp_pre-x2_tmp==0
             yawB_pre = yawB_tmp;
         else
             yawB_pre = atan2(y2_tmp_pre-y2_tmp,x2_tmp_pre-x2_tmp);
         end
         [X,Y,t1,t2,t3] = getlimit([yawA_final,yawB_pre],[0,x2_tmp],[0,y2_tmp]);
         pv1 = 1;
         pv2 = v2(i);
         field1 = zeros(size(X,2),size(Y,2));
         field2 = zeros(size(X,2),size(Y,2));
         if all(X == 0) && all(Y == 0) && length(X)==1
             Field(i) = 0;
         else
             for j = 1:1:size(X,2)
                 for m = 1:1:size(Y,2)
                     pd1 = (X(j).*X(j)) + (Y(m).*Y(m));
                     pd2 = (X(j)-x2_tmp).*(X(j)-x2_tmp) + (Y(m)-y2_tmp).*(Y(m)-y2_tmp);
                     [angel1, cosangle1] = getAngle(X(j),Y(m),0,0,yawA_final);
                     field1(j,m) = pv1 ./ pd1 .* cosangle1.* cosangle1;
                     [angel2, cosangle2] = getAngle(X(j),Y(m),x2_tmp,y2_tmp,yawB_tmp);
                     field2(j,m) = pv2 ./ pd2 .* cosangle2.* cosangle2;
                 end
             end
             Field(i) = sum(field1(~isnan(field1) & ~isinf(field1))) + sum(field2(~isnan(field2) & ~isinf(field2)));
         end
          
      elseif abs(v2(i)) < 1   % If only the other car is stationary
         B_tmp_final = Rotation_matrix * ([x2(time_length); y2(time_length)] - Translation_matrix);
         x2_tmp_final = B_tmp_final(1); y2_tmp_final = B_tmp_final(2); 
         if y2_tmp_final-y2_tmp==0 && x2_tmp_final-x2_tmp==0
             yawB_final = yawB_tmp;
         else
             yawB_final = atan2(y2_tmp_final-y2_tmp,x2_tmp_final-x2_tmp);
         end     
         A_tmp_pre = Rotation_matrix * ([x1(pre_time); y1(pre_time)] - Translation_matrix);
         x1_tmp_pre = A_tmp_pre(1); y1_tmp_pre = A_tmp_pre(2); 
         if y1_tmp_pre==0 && x1_tmp_pre==0
             yawA_pre = yawA_tmp;
         else
             yawA_pre = atan2(y1_tmp_pre-0,x1_tmp_pre-0); 
         end 
         [X,Y,t1,t2,t3] = getlimit([yawA_pre,yawB_final],[0,x2_tmp],[0,y2_tmp]);
         pv1 = v1(i);
         pv2 = 1;
         field1 = zeros(size(X,2),size(Y,2));
         field2 = zeros(size(X,2),size(Y,2));
         if all(X == 0) && all(Y == 0) && length(X)==1
             Field(i) = 0;
         else
             for j = 1:1:size(X,2)
                 for m = 1:1:size(Y,2)
                     pd1 = (X(j).*X(j)) + (Y(m).*Y(m));
                     pd2 = (X(j)-x2_tmp).*(X(j)-x2_tmp) + (Y(m)-y2_tmp).*(Y(m)-y2_tmp);
                     [angel1, cosangle1] = getAngle(X(j),Y(m),0,0,0);
                     field1(j,m) = pv1 ./ pd1 .* cosangle1.* cosangle1;
                     [angel2, cosangle2] = getAngle(X(j),Y(m),x2_tmp,y2_tmp,yawB_final);
                     field2(j,m) = pv2 ./ pd2 .* cosangle2.* cosangle2;
                 end
             end
             Field(i) = sum(field1(~isnan(field1) & ~isinf(field1))) + sum(field2(~isnan(field2) & ~isinf(field2)));
         end
          
      else   % If neither car is stationary   
          A_tmp_pre = Rotation_matrix * ([x1(pre_time); y1(pre_time)] - Translation_matrix);
          x1_tmp_pre = A_tmp_pre(1); y1_tmp_pre = A_tmp_pre(2); 
          if y1_tmp_pre==0 && x1_tmp_pre==0
              yawA_pre = yawA_tmp;
          else
              yawA_pre = atan2(y1_tmp_pre-0,x1_tmp_pre-0); 
          end
          B_tmp_pre = Rotation_matrix * ([x2(pre_time); y2(pre_time)] - Translation_matrix);
          x2_tmp_pre = B_tmp_pre(1); y2_tmp_pre = B_tmp_pre(2); 
          if y2_tmp_pre-y2_tmp==0 && x2_tmp_pre-x2_tmp==0
              yawB_pre = yawB_tmp;
          else
              yawB_pre = atan2(y2_tmp_pre-y2_tmp,x2_tmp_pre-x2_tmp);
          end
         [X,Y,t1,t2,t3] = getlimit([yawA_pre,yawB_pre],[0,x2_tmp],[0,y2_tmp]);
         pv1 = v1(i);
         pv2 = v2(i);
         field1 = zeros(size(X,2),size(Y,2));
         field2 = zeros(size(X,2),size(Y,2));
         if all(X == 0) && all(Y == 0) && length(X)==1
             Field(i) = 0;
         else
             for j = 1:1:size(X,2)
                  for m = 1:1:size(Y,2)
                      pd1 = (X(j).*X(j)) + (Y(m).*Y(m));
                      pd2 = (X(j)-x2_tmp).*(X(j)-x2_tmp) + (Y(m)-y2_tmp).*(Y(m)-y2_tmp);
                      [angel1, cosangle1] = getAngle(X(j),Y(m),0,0,0);
                      field1(j,m) = pv1 ./ pd1 .* cosangle1.* cosangle1;
                      [angel2, cosangle2] = getAngle(X(j),Y(m),x2_tmp,y2_tmp,yawB_tmp);
                      field2(j,m) = pv2 ./ pd2 .* cosangle2.* cosangle2;   
                  end
             end
             Field(i) = sum(field1(~isnan(field1) & ~isinf(field1))) + sum(field2(~isnan(field2) & ~isinf(field2)));
         end
      end
      
end

%% Calculate the interaction strength mean
if flag == 0
    mean_SAB = mean(Field);
else
    mean_SAB = sum(Field)./size(x1, 1);
end

disp('mean_SAB:');
disp(mean_SAB);