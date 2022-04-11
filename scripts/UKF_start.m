%----------------------------------------------------------------
% Template created for the course SD2231 by Mikael Nybacka 2019
% Following file is the start file for the state estimation using
% Uncented Kalman Filter (UKF).
%----------------------------------------------------------------
clear all;
close all;
clc;
addpath('scripts')
addpath('logged_data')
disp(' ');

% Set global variables so that they can be accessed from other matlab
% functions and files
global lf lr Cf Cr mass Iz vbox_file_name

%----------------------------
% LOAD DATA FROM VBOX SYSTEM
%----------------------------
%Test for SD2231 20180912

%Cloudy 18 degrees celsius
%2.8 bar in all four tyres
%Two persons in the front

%vbox_file_name='S90__035.VBO';   %Standstill

vbox_file_name='S90__036.VBO';   %Circular driving to the left, radius=8m
%vbox_file_name='S90__038.VBO';  %Slalom, v=30km/h
%vbox_file_name='S90__040.VBO';  %Step steer to the left, v=100km/h
%vbox_file_name='S90__041.VBO';  %Frequency sweep, v=50km/h


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
Ratio=16.3;         % Steering gear ratio
Cf=100000;          % Lateral stiffness front axle (N)
Cr=100000;          % Lateral stiffness rear axle (N)
Lx_relax=0.05;      % Longitudinal relaxation lenth of tyre (m)
Ly_relax=0.15;      % Lateral relaxation lenth of tyre (m)
Roll_res=0.01;      % Rolling resistance of tyre
rollGrad=4.5;       % rollangle deg per latacc
rx=0.29;         % distance from CoG to IMU x-axle
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
yawRate_VBOX    = vbo.channels(1, 56).data.*(-pi/180);%VBOX z-axis is pointing downwards, hence (-)
vx_VBOX         = vbo.channels(1, 54).data./3.6;
vy_VBOX         = vbo.channels(1, 52).data./3.6;
SteerAngle      = vbo.channels(1, 39).data./Ratio;
ax_VBOX         = vbo.channels(1, 57).data.*g;
ay_VBOX         = vbo.channels(1, 58).data.*g;
Beta_VBOX       = (vy_VBOX + rx*yawRate_VBOX)./vx_VBOX;

n = length(Time);
dt = Time(2)-Time(1);

%----------------------------------------------
% SET MEASUREMENT AND PROCESS NOICE COVARIANCES
%----------------------------------------------
% Use as starting value 0.1 for each of the states in Q matrix
Q=

% Use as starting value 0.01 for each of the measurements in R matrix
R=

%--------------------------------------------------
% SET INITIAL STATE AND STATE ESTIMATION COVARIANCE
%--------------------------------------------------
x_0=
P_0= 


%-----------------------
% INITIALISING VARIABLES
%-----------------------


%Parameters that might be needed in the measurement and state functions are added to predictParam
predictParam.dt=dt; 

% Handles to state and measurement model functions.
state_func_UKF = @Vehicle_state_eq;
meas_func_UKF = @Vehicle_measure_eq;

%-----------------------
% FILTERING LOOP FOR UKF 
%-----------------------
disp(' ');
disp('Filtering the signal with UKF...');

for i = 2:n


    % ad your predict and update functions, see the scripts ukf_predict1.m
    % and ukf_update1.m

    
    if i==round(n/4)
        disp(' ');
        disp('1/4 of the filtering done...');
        disp(' ');
    end
    if i==round(n/2)
        disp(' ');
        disp('1/2 of the filtering done...');
        disp(' ');
    end
    if i==round(n*(3/4))
        disp(' ');
        disp('3/4 of the filtering done... Stay tuned for the results...');
        disp(' ');
    end
end

%----------------------------------------
% CALCULATE THE SLIP ANGLE OF THE VEHICLE
%----------------------------------------


%---------------------------------------------------------
% CALCULATE THE ERROR VALES FOR THE ESTIMATE OF SLIP ANGLE
%---------------------------------------------------------
Beta_VBOX_smooth=smooth(Beta_VBOX,0.01,'rlowess'); 
[e_beta_mean,e_beta_max,time_at_max,error] = errorCalc(YOUR BETA',Beta_VBOX_smooth);
disp(' ');
fprintf('The MSE of Beta estimation is: %d \n',e_beta_mean);
fprintf('The Max error of Beta estimation is: %d \n',e_beta_max);

%-----------------
% PLOT THE RESULTS
%-----------------

