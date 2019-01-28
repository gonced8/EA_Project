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
kalman_quaternion = zeros(length(time),4);
kalman_quaternion(1,:) = q_0;
kalman_omega = zeros(length(time),3);
kalman_omega(1,:) = beta_0;
kalman_bias = zeros(length(time),3);
kalman_bias(1,:) = beta_0;
kalman_sigma = zeros(length(time),3);
kalman_sigma(1,:) = [P_0(1,1),P_0(2,2),P_0(3,3)];
kalman_Q = zeros(length(time), length(P_0), length(P_0));
kalman_Q(1,:,:) = Q_0;

% Attitude error
MEKF_q_e = zeros(size(kalman_quaternion));
MEKF_euler_e = zeros(length(kalman_quaternion),3);
MEKF_euler = zeros(length(kalman_quaternion),3);

npoints = 51;
alpha = linspace(0, 1, npoints);
beta = 1;

results = repmat(struct('alpha', 0, ...
                        'kalman_quaternion', kalman_quaternion, ...
                        'kalman_omega', kalman_omega, ...
                        'kalman_bias', kalman_bias, ...
                        'kalman_sigma', kalman_sigma, ...
                        'kalman_Q', kalman_Q, ...
                        'MEKF_q_e', MEKF_q_e, ...
                        'MEKF_euler_e', MEKF_euler_e, ...
                        'MEKF_euler', MEKF_euler), ...
                        npoints, 1 );
                    
clearvars kalman_quaternion kalman_omega kalman_bias kalman_sigma kalman_Q ...
          MEKF_q_e MEKF_euler_e MEKF_euler

disp('Start');
for i = 1:npoints
    fprintf('alpha = %.2f\n', alpha(i));
    
    results(i).alpha = alpha(i);
    
    AHRS = MEKF('q', q_0, 'bias', beta_0, 'P', P_0,...
    'sigma_acc', sigma_acc, 'sigma_mag', sigma_mag, 'sigma_opti', sigma_opti, ...
    'sigma_w', sigma_w, 'sigma_v', sigma_v, 'Q', Q_0, 'R', R_0);

    for k = 2 : length(time)
        dt = time(k) - time(k-1);

        AHRS.UPDATE(dt, gyroscope(k,:), accelerometer(k,:), magnetometer(k,:), alpha(i), beta);

        results(i).kalman_quaternion(k,:) = AHRS.q;
        results(i).kalman_omega(k,:) = AHRS.omega;
        results(i).kalman_bias(k,:) = AHRS.bias;
        results(i).kalman_sigma(k,:) = [sqrt(AHRS.P(1,1)),sqrt(AHRS.P(2,2)),sqrt(AHRS.P(3,3))];
        results(i).kalman_Q(k,:,:) = AHRS.Q;
    end

    for k = 1 : length(time)
        results(i).MEKF_q_e(k,:) = quatProd(optitrack(k,:)', quatConj(results(i).kalman_quaternion(k,:)'))';
        results(i).MEKF_euler_e(k,:) = quatToEuler( results(i).MEKF_q_e(k,:) );
        results(i).MEKF_euler(k,:) = quatToEuler( results(i).kalman_quaternion(k,:) );
    end
    
    clearvars AHRS
end
disp('Done');

% save('results.mat', 'time', 'results', '-v7.3');
% disp('Saved results.mat');
%% END OF CODE