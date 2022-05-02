function x_n = Vehicle_state_eq(x,param)
% ADDME Dynamic model function
%    x = the states
%    param = parameters that you might need, such as vehicle parameters.
%    NEED SteerAngle in rad
%    NEED dt
global lf lr mass Iz Cf Cr

vx=x(1,:);
vy=x(2,:);
psi=x(3,:);

alpha12=atan((vy+lf*psi)./vx)-param.SteerAngle;
alpha34=atan((vy-lr*psi)./vx);
F12 = -Cf*alpha12;
F34 = -Cr*alpha34;

f_x_1= -F12.*sin(param.SteerAngle)/mass + psi.*vy;
f_x_2=(F34+F12.*cos(param.SteerAngle))/mass -psi.*vx;
f_x_3=(lf*F12.*cos(param.SteerAngle)-lr*F34)/Iz;

% f_x = [(x(3,:).*x(2,:)+Cf*sin(SteerAngle)/mass*atan((x(2,:)+x(3,:)*lf)./x(1,:))-Cf/mass*SteerAngle.*sin(SteerAngle));
%        -Cr*atan((x(2,:)-x(3,:)*lr)./x(1,:))/mass - Cf*cos(SteerAngle).*(atan((x(2,:)+x(3,:)*lf)./x(1,:))-SteerAngle)/mass -x(3,:).*x(1,:);
%        -lf/Iz*Cf*cos(SteerAngle).*(atan((x(2,:)+x(3,:)*lf)./x(1,:))-SteerAngle)+lr/Iz*Cr*atan((x(2,:)-x(3,:)*lr)./x(1,:));];
f_x=[f_x_1;
     f_x_2;
     f_x_3];

% Integrate using Runge Kutta (in the script folder) or simple euler forward

f = @(x)[f_x(1,:);f_x(2,:);f_x(3,:)];
x_n = rk4(f,param.dt,x(1:3,:));