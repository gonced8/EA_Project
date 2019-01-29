clear
close all

%% Import data logged
LOG_NAME = 'P0167_20181211_1439'; %ANT-1 endurance
LOG_FOLDER = 'logs';
load([pwd filesep LOG_FOLDER filesep LOG_NAME]);

%% Parse data
time_sensors = sensor_combined_0.timestamp;                                %[s]
gyroscope = sensor_combined_0.gyro_rad;                                    %[rad/s]

clearvars -except ...
    time_sensors gyroscope

%% Plot data to identify a subset
fig=figure;
hax=axes; 
hold on
plot(gyroscope)
grid on

start = 20000;
stop = 140000;

line([start start],get(hax,'YLim'),'Color','red','LineStyle','--')
line([stop stop],get(hax,'YLim'),'Color','red','LineStyle','--')
hold off

%% Get the subsets
time_sensors = time_sensors(start:stop);
gyroscope = gyroscope(start:stop,:);

%% Interpolate the time vector to make it uniform and interpolate the 
% sensors consequently
dt = mode(diff(time_sensors));
time = time_sensors(1):dt:time_sensors(end);

gyroscope = interp1(time_sensors, gyroscope, time);

%% Calculate output angle for each sample
theta = cumtrapz(gyroscope, 1)*dt;

%% Calculate Allan variance
maxNumM = 1/dt;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*dt;

avar = zeros(numel(m), size(theta,2));
for i = 1:length(m)
    avar(i,:) = sum( ...
        (theta(1+2*m(i):L,:) - 2*theta(1+m(i):L-m(i),:) + theta(1:L-2*m(i),:)).^2, 1)./ ...
        (2*tau(i)^2 * (L - 2*m(i)));
end

%% Allan deviation
adev = sqrt(avar);
adev = mean(adev, 2);

ti = find(tau>1e-1, 1);
tau = tau(ti:end);
adev = adev(ti:end);

figure
hold on
plot(tau, adev)
title('Allan Deviation')
xlabel('\tau');
ylabel('\sigma(\tau)')
grid on
axis equal
set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')

%% Angle random walk
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);
plot(tau, lineN, '--', tauN, N, 'o')
title('Allan Deviation with Angle Random Walk')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma_N')
text(tauN, N, 'N')
grid on
axis equal
set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')

%% Rate random walk
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);
plot(tau, lineK, '--', tauK, K, 'o')
title('Allan Deviation with Rate Random Walk')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma_K')
text(tauK, K, 'K')
grid on
axis equal
set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')

clearvars -except N K