% Comparison Model SIFM
%% Prejudgment, all models are the same
flag = 0;
if size(x1, 1) > size(x2, 1)
    flag = 1;
end

if flag == 0
    time_length = size(x1, 1);
else
    time_length = size(x2, 1);
end
%% Calculate relative distance
R2 = (x1(1:time_length)-x2(1:time_length)).*(x1(1:time_length)-x2(1:time_length))+(y1(1:time_length)-y2(1:time_length)).*(y1(1:time_length)-y2(1:time_length));

%% Calculate the SIFM output
cos_yawA = cos(yawA(1:time_length));
cos_yawB = cos(yawB(1:time_length));
cos_yawA(cos_yawA<0) = 0;
cos_yawB(cos_yawB<0) = 0;

a = 5.102; % Parameters given in the reference.
b = 0.748;
c = 0.087;

p = 1 - exp(-(a.*(cos_yawA+c).*(cos_yawB+c)./R2).^b);

if flag == 0
    mean_p = mean(p);
else
    mean_p = sum(p)/size(x1, 1);
end