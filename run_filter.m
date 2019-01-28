%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MEKF                                %
% Author: M. Giurato                  %
% Date: 06/12/18                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clearvars
clc

%% Import data logged
LOG_NAME = '20181211_1459'; %ANT-1 C.L. identification

LOG_FOLDER = 'logs';
load([pwd filesep LOG_FOLDER filesep LOG_NAME]);

%% Parse data
time_sensors = sensor_combined_0.timestamp;                                %[s]

gyroscope = sensor_combined_0.gyro_rad;                                    %[rad/s]
accelerometer = sensor_combined_0.accelerometer_m_s2;                      %[m/s^2]
magnetometer = sensor_combined_0.magnetometer_ga;                          %[G]

time_optitrack = att_pos_mocap_0.timestamp;                                %[s]
optitrack = [att_pos_mocap_0.q(:,2:4), att_pos_mocap_0.q(:,1)];            %[1]

time_att = vehicle_attitude_0.timestamp;                                   %[s]
att_q = [vehicle_attitude_0.q(:,2:4), vehicle_attitude_0.q(:,1)];          %[1]
omega = [vehicle_attitude_0.rollspeed vehicle_attitude_0.pitchspeed vehicle_attitude_0.yawspeed];

% Interpolate data in order to have a fixed timestep
dt = mode(diff(time_sensors));
time = (time_sensors(1):dt:time_sensors(end))';

gyroscope = interp1(time_sensors, gyroscope, time);
accelerometer = interp1(time_sensors, accelerometer, time);
magnetometer = interp1(time_sensors, magnetometer, time);
optitrack = interp1(time_optitrack, optitrack, time);
att_q = interp1(time_att, att_q, time);
omega = interp1(time_att, omega, time);

clearvars -except ...
    time_sensors accelerometer gyroscope magnetometer ...
    time_optitrack optitrack ...
    time_att_q att_q omega ...
    time

%% Multiplicative Extended Kalman Filter
% Tuning
%Sigma accelerometer
sigma_acc = 1e-2;
%Sigma magnetometer
sigma_mag = 1e-2;
%Sigma optitrack 
sigma_opti = 1e-2;
%Sigma angular random walk (ARW)
sigma_v = 1e-3;
%Sigma rate random walk (RRW)
sigma_w = 1e-4;

% q_0 = [0 0 0 1]';
q_0 = init_q(accelerometer(1,:), magnetometer(1,:), [1, 1]);
beta_0 = gyroscope(1,:)';
P_0 = 1e-4*eye(6);

dt0 = time(2) - time(1);
Q_0 = [(sigma_v^2*dt0+1/3*sigma_w^2*dt0^3)*eye(3)       (1/2*sigma_w^2*dt0^2)*eye(3);
       (1/2*sigma_w^2*dt0^2)*eye(3)                     (sigma_w^2*dt0)*eye(3)      ];
Q_0 = 100*Q_0;
   
R_0 = [sigma_acc*eye(3)         zeros(3);
            zeros(3)        sigma_mag*eye(3)];

%Attitude Estimator
kalman_quaternion1 = zeros(length(time),4);
kalman_quaternion1(1,:) = q_0;
kalman_omega1 = zeros(length(time),3);
kalman_omega1(1,:) = beta_0;
kalman_bias1 = zeros(length(time),3);
kalman_bias1(1,:) = beta_0;
kalman_sigma1 = zeros(length(time),3);
kalman_sigma1(1,:) = [P_0(1,1),P_0(2,2),P_0(3,3)];
kalman_Q1 = zeros(length(time), length(P_0), length(P_0));
kalman_Q1(1,:,:) = Q_0;

AHRS1 = MEKF('q', q_0, 'bias', beta_0, 'P', P_0,...
    'sigma_acc', sigma_acc, 'sigma_mag', sigma_mag, 'sigma_opti', sigma_opti, ...
    'sigma_w', sigma_w, 'sigma_v', sigma_v, 'Q', Q_0, 'R', R_0);

kalman_quaternion2 = kalman_quaternion1;
kalman_omega2 = kalman_omega1;
kalman_bias2 = kalman_bias1;
kalman_sigma2 = kalman_sigma1;
kalman_Q2 = kalman_Q1;

alpha = 0.99;
AHRS2 = MEKF('q', q_0, 'bias', beta_0, 'P', P_0,...
    'sigma_acc', sigma_acc, 'sigma_mag', sigma_mag, 'sigma_opti', sigma_opti, ...
    'sigma_w', sigma_w, 'sigma_v', sigma_v, 'Q', Q_0, 'R', R_0);

for k = 2 : length(time)
    dt = time(k) - time(k-1);
    
    AHRS1.UPDATE(dt, gyroscope(k,:), accelerometer(k,:), magnetometer(k,:), 1);
    AHRS2.UPDATE(dt, gyroscope(k,:), accelerometer(k,:), magnetometer(k,:), alpha);
    
    kalman_quaternion1(k,:) = AHRS1.q;
    kalman_omega1(k,:) = AHRS1.omega;
    kalman_bias1(k,:) = AHRS1.bias;
    kalman_sigma1(k,:) = [sqrt(AHRS1.P(1,1)),sqrt(AHRS1.P(2,2)),sqrt(AHRS1.P(3,3))];
    kalman_Q1(k,:,:) = AHRS1.Q;

    kalman_quaternion2(k,:) = AHRS2.q;
    kalman_omega2(k,:) = AHRS2.omega;
    kalman_bias2(k,:) = AHRS2.bias;
    kalman_sigma2(k,:) = [sqrt(AHRS2.P(1,1)),sqrt(AHRS2.P(2,2)),sqrt(AHRS2.P(3,3))]; 
    kalman_Q2(k,:,:) = AHRS2.Q;
end

%% Attitude error
MEKF1_q_e = zeros(size(kalman_quaternion1));
MEKF1_euler_e = zeros(length(kalman_quaternion1),3);
MEKF1_euler = zeros(length(kalman_quaternion1),3);

MEKF2_q_e = zeros(size(kalman_quaternion2));
MEKF2_euler_e = zeros(length(kalman_quaternion2),3);
MEKF2_euler = zeros(length(kalman_quaternion2),3);

optitrack_euler = zeros(length(kalman_quaternion1),3);

%OB_q_e = zeros(size(kalman_quaternion));
%OB_euler_e = zeros(length(kalman_quaternion),3);
%OB_euler = zeros(length(kalman_quaternion),3);

for k = 1 : length(time)
    MEKF1_q_e(k,:) = quatProd(optitrack(k,:)', quatConj(kalman_quaternion1(k,:)'))';
    MEKF1_euler_e(k,:) = quatToEuler( MEKF1_q_e(k,:) );
    MEKF1_euler(k,:) = quatToEuler( kalman_quaternion1(k,:) );
    
    MEKF2_q_e(k,:) = quatProd(optitrack(k,:)', quatConj(kalman_quaternion2(k,:)'))';
    MEKF2_euler_e(k,:) = quatToEuler( MEKF2_q_e(k,:) );
    MEKF2_euler(k,:) = quatToEuler( kalman_quaternion2(k,:) );
    
    optitrack_euler(k,:) = quatToEuler( optitrack(k,:) );
    
    %OB_q_e(k,:) = quatProd(optitrack(k,:)', quatConj(att_q(k,:)'))';
    %OB_euler_e(k,:) = quatToEuler( OB_q_e(k,:) );
    %OB_euler(k,:) = quatToEuler( att_q(k,:) );
end


%% END OF CODE