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
time = (max(time_sensors(1), time_optitrack(1)) ...
        :dt: ...
        150)';%min(time_sensors(end), time_optitrack(end)))';

gyroscope = interp1(time_sensors, gyroscope, time);
accelerometer = interp1(time_sensors, accelerometer, time);
magnetometer = interp1(time_sensors, magnetometer, time);
optitrack = interp1(time_optitrack, optitrack, time);     % ended with NaN
att_q = interp1(time_att, att_q, time);
omega = interp1(time_att, omega, time);

clearvars -except ...
    time_sensors accelerometer gyroscope magnetometer ...
    time_optitrack optitrack ...
    time_att_q att_q omega ...
    time dt

%% Multiplicative Extended Kalman Filter
% Tuning
%Sigma accelerometer
sigma_acc = 0.1237; 
%Sigma magnetometer
sigma_mag = 0.0560;
%Sigma optitrack 
sigma_opti = 1e-2;
%Sigma angular random walk (ARW)
sigma_v = 9.486e-3; 
%Sigma rate random walk (RRW)
sigma_w = 5.621e-5; 


% %Sigma accelerometer
% sigma_acc = 1e-2;
% %Sigma magnetometer
% sigma_mag = 1e-2;
% %Sigma optitrack 
% sigma_opti = 1e-2;
% %Sigma angular random walk (ARW)
% sigma_v = 1e-3;
% %Sigma rate random walk (RRW)
% sigma_w = 1e-4;


%q_0 = [0 0 0 1]';
q_0 = init_q(accelerometer(1,:), magnetometer(1,:), [1, 1]);
beta_0 = gyroscope(1,:)';
P_0 = 1e-4*eye(6);

err = 0.1;
Q_0 = [((err*sigma_v)^2*dt+1/3*(err*sigma_w)^2*dt^3)*eye(3)       (1/2*(err*sigma_w)^2*dt^2)*eye(3) ;
       (1/2*(err*sigma_w)^2*dt^2)*eye(3)                          ((err*sigma_w)^2*dt)*eye(3)      ];
%Q_0 = 100*Q_0;
   
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

ff = [true];     % forgetting factor flag
value = [-1];

% update Q matrix with forgetting factor
npoints = 51;
alpha_value = linspace(0, 1, npoints);
value = [value, alpha_value];
beta = 1;
ff = [ff, true(1,length(alpha_value))];

% update Q matrix with estimated value
npoints = 55;
window_max = 10000;
window_value = logspace(0, log10(window_max), npoints);
window_value = ceil(window_value); % value must be an integer.
window_value = unique(window_value); % Remove duplicates.
value = [value, window_value];
ff = [ff, false(1,length(window_value))];
dd = zeros(length(time)-1, length(Q_0), length(Q_0));
%

npoints = length(ff);

results = repmat(struct('ff', false, ...
                        'value', 0, ...
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
          MEKF_q_e MEKF_euler_e MEKF_euler alpha_value window_value

disp('Start');
for i = 1:npoints
    fprintf('value = %d\n', value(i));
    
    results(i).ff = ff(i);
    results(i).value = value(i);
    
    AHRS = MEKF('q', q_0, 'bias', beta_0, 'P', P_0,...
    'sigma_acc', sigma_acc, 'sigma_mag', sigma_mag, 'sigma_opti', sigma_opti, ...
    'sigma_w', sigma_w, 'sigma_v', sigma_v, 'Q', Q_0, 'R', R_0);

    for k = 2 : length(time)
        dt = time(k) - time(k-1);

        if ff(i)
            AHRS.UPDATE(dt, gyroscope(k,:), accelerometer(k,:), magnetometer(k,:), value(i), 1);            
        else
            if k>2
                Edd = mean(dd(max(1,k-1-value(i)):k-2, :, :), 1);
                Edd = reshape(Edd, size(Edd,2), size(Edd,3));
                AHRS.UPDATE(dt, gyroscope(k,:), accelerometer(k,:), magnetometer(k,:), Edd, 1);
            else
                AHRS.UPDATE(dt, gyroscope(k,:), accelerometer(k,:), magnetometer(k,:), 1, 1);
            end
            dd(k-1,:,:) = AHRS.d*AHRS.d';
        end
        
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

% correct theoretical Q initial matrix Q
i = find(extractfield(results, 'value')==-1, 1);
if ~isempty(i)
    results(i).kalman_Q(1,:,:) = results(1).kalman_Q(2,:,:);  
end

disp('Done');

save('results150.mat', 'time', 'results', 'optitrack', 'omega', '-v7.3');
disp('Saved results.mat');

%% END OF CODE