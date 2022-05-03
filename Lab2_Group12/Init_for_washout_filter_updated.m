%----------------------------------------------------------------
% Template created for the course SD2231 by Mikael Nybacka 2019
% Following file is the start file for the Simulink implementation 
% of the integration, model-based, and washout filter.
%----------------------------------------------------------------
clear all;
close all;
clc;
addpath('scripts')
addpath('logged_data')
disp(' ');


% Set global variables so that they can be accessed from other matlab
% functions and files
global vbox_file_name

%----------------------------
% LOAD DATA FROM VBOX SYSTEM
%----------------------------
%Test for SD2231 20180912

%Cloudy 18 degrees celsius
%2.8 bar in all four tyres
%Two persons in the front

%vbox_file_name='S90__035.VBO';   %Standstill

%vbox_file_name='S90__036.VBO';   %Circular driving to the left, radius=8m
%vbox_file_name='S90__038.VBO';  %Slalom, v=30km/h
%vbox_file_name='S90__040.VBO';  %Step steer to the left, v=100km/h
vbox_file_name='S90__041.VBO';  %Frequency sweep, v=50km/h


vboload
%  Channel 1  = satellites
%  Channel 2  = time
%  Channel 3  = latitude
%  Channel 4  = longitude
%  Channel 5  = velocity kmh
%  Channel 6  = heading
%  Channel 7  = height
%  Channel 8  = vertical velocity kmh
%  Channel 9  = long accel g
%  Channel 10 = lat accel g
%  Channel 11 = glonass_sats
%  Channel 12 = gps_sats
%  Channel 13 = imu kalman filter status
%  Channel 14 = solution type
%  Channel 15 = velocity quality
%  Channel 16 = event 1 time
%  Channel 17 = latitude_raw
%  Channel 18 = longitude_raw
%  Channel 19 = speed_raw
%  Channel 20 = heading_raw
%  Channel 21 = height_raw
%  Channel 22 = vertical_velocity_raw
%  Channel 23 = rms_hpos
%  Channel 24 = rms_vpos
%  Channel 25 = rms_hvel
%  Channel 26 = rms_vvel
%  Channel 27 = poscov_xx
%  Channel 28 = poscov_yy
%  Channel 29 = poscov_zz
%  Channel 30 = velcov_xx
%  Channel 31 = velcov_yy
%  Channel 32 = velcov_zz
%  Channel 33 = t1
%  Channel 34 = accrpedlln 
%  Channel 35 = engn
%  Channel 36 = ptgearact
%  Channel 37 = pttqatw_fl 
%  Channel 38 = pttqatw_fr 
%  Channel 39 = swa
%  Channel 40 = brkpedlpsd 
%  Channel 41 = vehspdlgt
%  Channel 42 = flwhlspd
%  Channel 43 = frwhlspd
%  Channel 44 = rlwhlspd
%  Channel 45 = rrwhlspd
%  Channel 46 = algt1
%  Channel 47 = alat1
%  Channel 48 = rollrate1
%  Channel 49 = yawrate1
%  Channel 50 = true_head
%  Channel 51 = slip_angle 
%  Channel 52 = lat._vel.
%  Channel 53 = roll_angle 
%  Channel 54 = lng._vel.
%  Channel 55 = slip_cog
%  Channel 56 = yawrate
%  Channel 57 = x_accel
%  Channel 58 = y_accel
%  Channel 59 = temp
%  Channel 60 = pitchrate
%  Channel 61 = rollrate
%  Channel 62 = z_accel
%  Channel 63 = roll_imu
%  Channel 64 = pitch_ang. 
%  Channel 65 = yaw_rate
%  Channel 66 = slip_fl
%  Channel 67 = slip_fr
%  Channel 68 = slip_rl
%  Channel 69 = slip_rr
%  Channel 70 = true_head2 
%  Channel 71 = head_imu
%  Channel 72 = pitch_imu
%  Channel 73 = pos.qual.
%  Channel 74 = lng_jerk
%  Channel 75 = lat_jerk
%  Channel 76 = head_imu2

%-----------------------------------
% SET VEHICLE DATA FOR THE VOLVO S90
%-----------------------------------
Rt=0.35;            % Tyre radius (m)
L=2.941;            % Wheel base (m)
lf=1.65;            % Distance from CoG to front axis (m)
lr=L-lf;            % Distance from CoG to rear axis (m)
mass=2010.5;        % Mass (kg)
Iz=3089;            % Yaw inertia (kg-m2)
tw=1.617;           % Track width (m)
h_cog = 0.570;      % Height of CoG above ground
Ratio=25;         % Steering gear ratio                         [tuned] 
Cf=175000;          % Lateral stiffness front axle (N)          [tuned] 
Cr=175000;          % Lateral stiffness rear axle (N)           [tuned] 
Lx_relax=0.05;      % Longitudinal relaxation lenth of tyre (m)
Ly_relax=0.15;      % Lateral relaxation lenth of tyre (m)
Roll_res=0.01;      % Rolling resistance of tyre
rollGrad=4.1;       % rollangle deg per latacc                  [tuned] 
rx=0.29;            % distance from CoG to IMU x-axle
ry=0;               % distance from CoG to IMU y-axle
rz=0;               % distance from CoG to IMU z-axle

Ts = 0.01;

%--------------------------------------     
% SET ENVIRONEMNTAL PARAMETERS FOR TEST
%--------------------------------------
Mu=0.85;            % Coefficient of friction
g=9.81;             % Gravity constant

%--------------------------------------------
% SET VARIABLES DATA FROM DATA READ FROM FILE
%--------------------------------------------

Time            = vbo.channels(1, 2).data-vbo.channels(1, 2).data(1,1);
yawRate_VBOX    = vbo.channels(1, 56).data.*(-pi/180); %VBOX z-axis is pointing downwards, hence (-)
vx_VBOX         = vbo.channels(1, 54).data./3.6;
vy_VBOX         = vbo.channels(1, 52).data./3.6;
SteerAngle      = vbo.channels(1, 39).data./Ratio;
ax_VBOX         = vbo.channels(1, 57).data.*g; %[m/s^2]
ay_VBOX         = vbo.channels(1, 58).data.*g;
Beta_VBOX       = (vy_VBOX + rx*yawRate_VBOX)./vx_VBOX; 

%% Model-based body side slip
vy_mod = vx_VBOX.*(lr*(lf+lr)*Cf*Cr-lf*Cf*mass*vx_VBOX.^2).*SteerAngle./((lf+lr)^2*Cf*Cr+mass*vx_VBOX.^2*(lr*Cr-lf*Cf)); %Lateral acceleration 
Kroll = rollGrad*pi/180;        %Converts from rad to deg 
T=0.15;                         %Tuned filter coefficient T 
a=0.05; %best cir 0.05, 0.05, 1, 0.05
b=0.5; %best cir 1.5, 0.1, 0.5, 0.1
Tlist=[Time,a*SteerAngle + b];  %Linear function for filter coefficient T, for task 1.f 
%Best value of T for each test in the respective order: 
%First test : T = 0.15 
%Second test : T = 0.01 
%Third test : T = 0.1 
%Forth test : T = 0.15 
%Converting the data into matrices with both time and the respective data 
vy_mod_struct = [Time,vy_mod];
vx_VBOX_struct = [Time,vx_VBOX];
vy_VBOX_struct = [Time,vy_VBOX];
yawRate_VBOX_struct = [Time, yawRate_VBOX];
ay_VBOX_struct = [Time, ay_VBOX];
Beta_VBOX_struct = [Time, Beta_VBOX]; 
SteerAngle_struct = [Time,SteerAngle];

threshold_vx = 1;   %Threshold for vx used in Simulink 
sim wash_out_filter.slx

%% Plots 

%Note : for the slalom test, you must change Time(1:end) to Time(1:end-1)
%for each plot function, except (Time, Beta_VBOX) one.
close all

%Plots for the estimators compared to true beta values 
figure
% subplot(2,1,1)
plot(Time, Beta_VBOX)
hold on
plot(Time(1:end), ans.beta_kin_est.Data)
plot(Time(1:end), ans.beta_mod_est.Data)
plot(Time(1:end),ans.beta_washout_est.Data)
plot(Time(1:end),ans.beta_washout_est_dis.Data)
legend('True Beta','Beta estimator kin', 'Beta estimator mod','Washout','discrete')
xlabel('Time [s]')
ylabel('Beta (rad)')
title('Estimation of side slip')
hold off 

%Plots for task 1.e, when comparing the dependencies between the errors for the
%estimators and the velocity 
% subplot(2,1,2)
% plot(Time(1:end),abs(ans.beta_kin_est.Data-Beta_VBOX(1:end)-Beta_VBOX(1:end)),'r')
% hold on
% plot(Time(1:end),abs(ans.beta_mod_est.Data-Beta_VBOX(1:end)-Beta_VBOX(1:end)),color=[0.92 0.69 0.125])
% plot(Time(1:end),abs(ans.beta_washout_est.Data-Beta_VBOX(1:end)-Beta_VBOX(1:end)),'m')
% xlabel("Time in s")
% ylabel("Beta in rad")
% yyaxis right
% plot(Time,vy_VBOX)
% ylabel("velocity x in m/s")
% legend('error kin', 'error mod','error washout', 'velocity')

%Plot for the washout filter with the filter coefficient T as a linear
%function (task 1.f) 
% figure
% plot(Time, Beta_VBOX)
% hold on
% plot(Time(1:end),ans.beta_washout_est.Data)
% plot(Time(1:end),ans.beta_washout_est_dis.Data)
% legend('True Beta','Washout','discrete')
% xlabel('Time [s]')
% ylabel('Beta (rad)')
% title('Estimation of side slip')
% hold off 
%---------------------------------------------------------
% CALCULATE THE ERROR VALES FOR THE ESTIMATE OF SLIP ANGLE
%---------------------------------------------------------
Beta_VBOX_smooth=smooth(Beta_VBOX,0.01,'rlowess'); 

% [e_beta_mean,e_beta_max,time_at_max,error] = errorCalc(ans.beta_kin_est.Data(1:end),Beta_VBOX_smooth(1:end)); 
% disp(' ');
% fprintf('The MSE of Beta kinematic estimation is: %d \n',e_beta_mean);
% fprintf('The Max error of Beta kinematic estimation is: %d \n',e_beta_max);
% 
% [e_beta_mean,e_beta_max,~,~] = errorCalc(ans.beta_mod_est.Data(1:end),Beta_VBOX_smooth(1:end));
% disp(' ');
% fprintf('The MSE of Beta model-bas~ed estimation is: %d \n',e_beta_mean);
% fprintf('The Max error of Beta model-based estimation is: %d \n',e_beta_max);
% 
% [e_beta_mean,e_beta_max,~,~] = errorCalc(ans.beta_washout_est.Data(1:end),Beta_VBOX_smooth(1:end));
% disp(' ');
% fprintf('The MSE of Beta washout estimation is: %d \n',e_beta_mean);
% fprintf('The Max error of Beta washout estimation is: %d \n',e_beta_max);

[e_beta_mean,e_beta_max,~,~] = errorCalc(ans.beta_washout_est_dis.Data(1:end),Beta_VBOX_smooth(1:end));
disp(' ');
fprintf('The MSE of Beta washout estimation is: %d \n',e_beta_mean);
fprintf('The Max error of Beta washout estimation is: %d \n',e_beta_max);
%'True Beta', 'Beta estimator kin', 'Beta estimator mod',
%,'Beta estimator washout filter'