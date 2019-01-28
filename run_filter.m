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

%q_0 = [0 0 0 1]';
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

%% Plot
figure
subplot(4,1,1)
hold on
title('MEKF - Quaternion')
plot(time, optitrack(:,1))
plot(time, kalman_quaternion1(:,1))
plot(time, kalman_quaternion2(:,1))
hold off
grid
ylabel('$q_1$','Interpreter','latex','fontsize',12.0)
subplot(4,1,2)
hold on
plot(time, optitrack(:,2))
plot(time, kalman_quaternion1(:,2))
plot(time, kalman_quaternion2(:,2))
hold off
grid
ylabel('$q_2$','Interpreter','latex','fontsize',12.0)
subplot(4,1,3)
hold on
plot(time, optitrack(:,3))
plot(time, kalman_quaternion1(:,3))
plot(time, kalman_quaternion2(:,3))
hold off
grid
ylabel('$q_3$','Interpreter','latex','fontsize',12.0)
subplot(4,1,4)
hold on
plot(time, optitrack(:,4))
plot(time, kalman_quaternion1(:,4))
plot(time, kalman_quaternion2(:,4))
hold off
ylabel('$q_4$','Interpreter','latex','fontsize',12.0)
grid
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend('Optitrack', 'MEKF (normal)', 'MEKF (updating Q)')

figure
subplot(3,1,1)
title('Estimated Euler angles')
hold on
plot(time, optitrack_euler(:,1))
plot(time, MEKF1_euler(:,1))
plot(time, MEKF2_euler(:,1))
hold off
ylim([-0.5;0.5])
grid
ylabel('$\phi [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
plot(time, optitrack_euler(:,2))
plot(time, MEKF1_euler(:,2))
plot(time, MEKF2_euler(:,2))
hold off
ylim([-0.5;0.5])
grid
ylabel('$\theta [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
plot(time, optitrack_euler(:,3))
plot(time, MEKF1_euler(:,3))
plot(time, MEKF2_euler(:,3))
hold off
grid
ylabel('$\psi [rad]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend('Optitrack', 'MEKF (normal)', 'MEKF (updating Q)')

figure
subplot(3,1,1)
title('Estimation error')
hold on
plot(time, MEKF1_euler_e(:,1))
plot(time, MEKF2_euler_e(:,1))
plot(time, 3*kalman_sigma1(:,1),'r--')
plot(time, -3*kalman_sigma1(:,1),'r--')
plot(time, 3*kalman_sigma2(:,1),'b--')
plot(time, -3*kalman_sigma2(:,1),'b--')
hold off
ylim([-0.5;0.5])
grid
ylabel('$\delta\phi [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
plot(time, MEKF1_euler_e(:,2))
plot(time, MEKF2_euler_e(:,2))
plot(time, 3*kalman_sigma1(:,2),'r--')
plot(time, -3*kalman_sigma1(:,2),'r--')
plot(time, 3*kalman_sigma2(:,2),'b--')
plot(time, -3*kalman_sigma2(:,2),'b--')
hold off
ylim([-0.5;0.5])
grid
ylabel('$\delta\theta [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
plot(time, MEKF1_euler_e(:,3))
plot(time, MEKF2_euler_e(:,3))
plot(time, 3*kalman_sigma1(:,3),'r--')
plot(time, -3*kalman_sigma1(:,3),'r--')
plot(time, 3*kalman_sigma2(:,3),'b--')
plot(time, -3*kalman_sigma2(:,3),'b--')
hold off
grid
ylabel('$\delta\psi [rad]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend('MEKF (normal) vs Optitrack','MEKF (updating Q) vs Optitrack')

figure
subplot(3,1,1)
title('Angular velocity')
hold on
plot(time, kalman_omega1(:,1))
plot(time, kalman_omega2(:,1))
plot(time, omega(:,1))
hold off
grid
ylabel('$p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
plot(time, kalman_omega1(:,2))
plot(time, kalman_omega2(:,2))
plot(time, omega(:,2))
hold off
grid
ylabel('$q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
plot(time, kalman_omega1(:,3))
plot(time, kalman_omega2(:,3))
plot(time, omega(:,3))
hold off
grid
ylabel('$r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend('MEKF (normal)', 'MEKF (updating Q)', 'OB')

figure
subplot(3,1,1)
title('Estimated bias')
hold on
plot(time, kalman_bias1(:,1))
plot(time, kalman_bias2(:,1))
hold off
grid
ylabel('$p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
plot(time, kalman_bias1(:,2))
plot(time, kalman_bias2(:,2))
hold off
grid
ylabel('$q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
plot(time, kalman_bias1(:,3))
plot(time, kalman_bias2(:,3))
hold off
grid
ylabel('$r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend('MEKF (normal)', 'MEKF (updating Q)')

figure
subplot(3,1,1)
title('Error angular velocities')
hold on;
plot(time, kalman_omega1(:,1)-omega(:,1), ...
     time, kalman_omega2(:,1)-omega(:,1));
grid
ylabel('$\mathrm{error} \; p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
plot(time, kalman_omega1(:,2)-omega(:,2), ...
     time, kalman_omega2(:,2)-omega(:,2));
grid;
ylabel('$\mathrm{error} \; q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
plot(time, kalman_omega1(:,3)-omega(:,3), ...
     time, kalman_omega2(:,3)-omega(:,3));
grid
ylabel('$\mathrm{error} \; r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend('MEKF (normal) vs OB', 'MEKF (updating Q) vs OB')

%% END OF CODE