%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PSD                                 %
% Author: M. Giurato                  %
% Date: 06/12/18                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clearvars
clc

%% Import data logged
LOG_NAME = 'P0167_20181211_1439'; %ANT-1 endurance
LOG_FOLDER = 'logs';
load([pwd filesep LOG_FOLDER filesep LOG_NAME]);

%% Parse data
time_sensors = sensor_combined_0.timestamp;                                %[s]
gyroscope = sensor_combined_0.gyro_rad;                                    %[rad/s]
accelerometer = sensor_combined_0.accelerometer_m_s2;                      %[m/s^2]
magnetometer = sensor_combined_0.magnetometer_ga;                          %[G]

clearvars -except ...
    time_sensors accelerometer gyroscope magnetometer

%% Plot data to identify a subset
fig=figure;
hax=axes; 
hold on
plot(accelerometer)
% plot(gyroscope)
% plot(magnetometer)
grid on

start = 3000;
stop = 150000;

line([start start],get(hax,'YLim'),'Color','red','LineStyle','--')
line([stop stop],get(hax,'YLim'),'Color','red','LineStyle','--')
hold off

%% Get the subsets
time_sensors = time_sensors(start:stop);
gyroscope = gyroscope(start:stop,:);
accelerometer = accelerometer(start:stop,:);
magnetometer = magnetometer(start:stop,:);

%% Calculate the mean values and depolarize the sensors
gyr_mean = mean(gyroscope);
acc_mean = mean(accelerometer);
mag_mean = mean(magnetometer);

gyroscope = gyroscope - gyr_mean;
accelerometer = accelerometer - acc_mean;
magnetometer = magnetometer - mag_mean;

%% Interpolate the time vector to make it uniform and interpolate the 
% sensors consequently
dt = mean(diff(time_sensors));
time = time_sensors(1):dt:time_sensors(end);

gyroscope = interp1(time_sensors, gyroscope, time);
accelerometer = interp1(time_sensors, accelerometer, time);
magnetometer = interp1(time_sensors, magnetometer, time);

%% Calculate variance of continuous time noise
sigmasq_gyr = var(gyroscope); 
sigmasq_acc = var(accelerometer);
sigmasq_mag = var(magnetometer);

sigma_gyr_k = mean(sqrt(sigmasq_gyr))
sigma_acc_k = mean(sqrt(sigmasq_acc))
sigma_mag_k = mean(sqrt(sigmasq_mag))

%% Compute the PSD
na = 100;

[Pxx_gyro_x, f_gyro_x] = pwelchrun(gyroscope(:,1), na, 1/dt, 'onesided');
[Pxx_gyro_y, f_gyro_y] = pwelchrun(gyroscope(:,2), na, 1/dt, 'onesided');
[Pxx_gyro_z, f_gyro_z] = pwelchrun(gyroscope(:,3), na, 1/dt, 'onesided');

figure
loglog(f_gyro_x, Pxx_gyro_x)
hold on
loglog(f_gyro_y, Pxx_gyro_y)
loglog(f_gyro_z, Pxx_gyro_z)
ax = gca;
ax.ColorOrderIndex = 1;
% loglog(f_gyro_x, sigmasq_gyr(1,1)*ones(size(f_gyro_x)))
% loglog(f_gyro_y, sigmasq_gyr(1,2)*ones(size(f_gyro_y)))
% loglog(f_gyro_z, sigmasq_gyr(1,3)*ones(size(f_gyro_z)))
hold off
grid
legend('X', 'Y', 'Z')
xlabel('Frequency [$Hz$]','Interpreter','latex','fontsize',12.0)
title('PSD - Gyroscope')
ylabel('$\left[\frac{(rad/s)^2}{Hz}\right]$','Interpreter','latex','fontsize',12.0)

[Pxx_acc_x, f_acc_x] = pwelchrun(accelerometer(:,1), na, 1/dt, 'onesided');
[Pxx_acc_y, f_acc_y] = pwelchrun(accelerometer(:,2), na, 1/dt, 'onesided');
[Pxx_acc_z, f_acc_z] = pwelchrun(accelerometer(:,3), na, 1/dt, 'onesided');

figure
loglog(f_acc_x, Pxx_acc_x)
hold on
loglog(f_acc_y, Pxx_acc_y)
loglog(f_acc_z, Pxx_acc_z)
ax = gca;
ax.ColorOrderIndex = 1;
% loglog(f_acc_x, sigmasq_acc(1,1)*ones(size(f_acc_x)))
% loglog(f_acc_y, sigmasq_acc(1,2)*ones(size(f_acc_y)))
% loglog(f_acc_z, sigmasq_acc(1,3)*ones(size(f_acc_z)))
hold off
grid
legend('X', 'Y', 'Z')
xlabel('Frequency [$Hz$]','Interpreter','latex','fontsize',12.0)
title('PSD - Accelerometer')
ylabel('$\left[\frac{(m/s^2)^2}{Hz}\right]$','Interpreter','latex','fontsize',12.0)

[Pxx_mag_x, f_mag_x] = pwelchrun(magnetometer(:,1), na, 1/dt, 'onesided');
[Pxx_mag_y, f_mag_y] = pwelchrun(magnetometer(:,2), na, 1/dt, 'onesided');
[Pxx_mag_z, f_mag_z] = pwelchrun(magnetometer(:,3), na, 1/dt, 'onesided');

figure
loglog(f_mag_x, Pxx_mag_x)
hold on
loglog(f_mag_y, Pxx_mag_y)
loglog(f_mag_z, Pxx_mag_z)
ax = gca;
ax.ColorOrderIndex = 1;
% loglog(f_mag_x, sigmasq_mag(1,1)*ones(size(f_mag_x)))
% loglog(f_mag_y, sigmasq_mag(1,2)*ones(size(f_mag_y)))
% loglog(f_mag_z, sigmasq_mag(1,3)*ones(size(f_mag_z)))
hold off
grid
legend('X', 'Y', 'Z')
xlabel('Frequency [$Hz$]','Interpreter','latex','fontsize',12.0)
title('PSD - Magnetometer')
ylabel('$\left[\frac{(G)^2}{Hz}\right]$','Interpreter','latex','fontsize',12.0)

%% END OF CODE