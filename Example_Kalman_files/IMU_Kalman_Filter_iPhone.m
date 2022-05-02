%----------------------------------------------------------------
% Program created for the course SD2231 by Mikael Nybacka 2018
% Following file is the start file for the state estimation using
% Kalman filter for estimating Roll-angle from IMU data of iPhone.
%----------------------------------------------------------------
clear;close all;clc;
addpath('logged_data')
disp(' ');

%----------------------------
% LOAD DATA FROM VBOX SYSTEM
%----------------------------
iPhoneData=load('iPhoneData.mat');

%iPhoneData.accdata is x, y, z accelerations of phone
%iPhoneData.acctime is time vector for accelerations on phone
%iPhoneData.ratedata is x, y, z rotational rates of phone
%iPhoneData.ratetime is time vector for rotational rates on phone
%iPhoneData.orientdata is x, y, z orientations in deg of phone
%iPhoneData.orienttime is time vector for orientations on phone
 
dt=0.098; % iPhone logs in 10Hz

Timeacc = iPhoneData.acctime(1:100,1);
Timerate = iPhoneData.ratetime(1:100,1);
Timeorient = iPhoneData.orienttime(1:100,1);
ax = iPhoneData.accdata(1:100,1);
ay = iPhoneData.accdata(1:100,1);
az = iPhoneData.accdata(1:100,3);
rollrate = iPhoneData.ratedata(1:100,2);
rollangle = -iPhoneData.orientdata(1:100,3);

rollRate_noise = 1.4341; % Roll-rate measurement noise (deg/sec)
angle_noise = 904.3045; % Roll-angle measurement noise (deg)
bias_noise = 2000; % Roll-rate bias noise, tunable

a = [1 dt; 
     0 1]; % transition matrix
b = [dt; 
     0]; % input matrix
c = [1 0]; % measurement matrix
x = [0;
     0]; % initial state vector
xhat = x; % initial state estimate

Sz = angle_noise^2; % Measurement noise covariance
Sw = [(rollRate_noise^2)*(dt^2) 0; 
      0 (bias_noise^2)]; % Process noise covariance
P = Sw; % Initial estimation covariance

% Initialize arrays for later plotting.
angle = []; % true angle array
anglehat = []; % estimated angle array
anglemeas = []; % measured angle array
biashat = []; % estimated bias array

for i = 1 : 1 : length(Timeacc)

    u = rollrate(i,1); % Roll rate

    y = atan((ax(i,1)/9.81)/(abs(az(i,1))/9.81))*(180/pi); % Measurement equation
    
    % Extrapolate the most recent state estimate to the present time.
    xhat = a * xhat + b * u;
    
    % Form the Innovation vector.
    Inn = y - c * xhat;
    
    % Compute the covariance of the Innovation.
    s = c * P * c' + Sz;
    
    % Form the Kalman Gain matrix.
    K = (a * P * c') / s;
    
    % Update the state estimate.
    xhat = xhat + K * Inn;
    
    % Compute the covariance of the estimation error.
    P = a * P * a' - ((a * P * c' )/ s) * c * P * a' + Sw;
    
    % Save some parameters for plotting later.
    angle = [angle; rollangle(i,1)];
    anglemeas = [anglemeas; y];
    anglehat = [anglehat; xhat(1)];
    biashat = [biashat; xhat(2)];
end

% Plot the results
close all;
t = Timeacc;
tr = Timerate;
to = Timeorient;

figure;
plot(to,angle,t,anglemeas,'r-',t,anglehat,'g-');
grid;
xlabel('Time (sec)');
ylabel('Angle (deg)');
title('Figure 1 - Angle (True(b), Measured(r), and Estimated(g))')

figure;
plot(t,angle-anglehat);
grid;
xlabel('Time (sec)');
ylabel('Angle Error (deg)');
title('Figure 2 - Angle Estimation Error');

figure;
plot(t,biashat);
grid;
xlabel('Time (sec)');
ylabel('Bias (deg/s)');
title('Figure 3 - Bias (Estimated)');

figure;
plot(t,ax,'-',t,ay,'r-',t,az,'g-');
grid;
xlabel('Time (sec)');
ylabel('Acceleration (m/s^2)');
title('Figure 4 - Ax, Ay, Az');


title('Figure 4 - x, y, z');