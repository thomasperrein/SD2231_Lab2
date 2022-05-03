function y_n = Vehicle_measure_eq(x,param)
% ADDME Measurement function
%    x = the states
%    param = parameters that you might need, such as vehicle parameters.
%    SteerAngle

global lf lr mass Cf Cr


vx=x(1,:);
vy=x(2,:);
psi=x(3,:);

alpha12=atan((vy+lf*psi)./vx)-param.SteerAngle;
alpha34=atan((vy-lr*psi)./vx);
F12 = -Cf*alpha12;
F34 = -Cr*alpha34;

y_n=[x(1,:);
       (F34 + F12.*cos(param.SteerAngle))/mass;
       x(3,:);];

% h = @(x)[h_x(1,:);h_x(2,:);h_x(3,:)];
% y_n = rk4(h,param.dt,x(1:3,:));