% Obtain data in double array for alpha vals and cell arrays for the rest from results.mat
get_data;


%% Plot
clear;

if ~exist('results') || ~exist('time')
    get_data;
end

% INPUTS -------- %
alpha_s = [0; 0.7; 1];  % -1 corresponds to not updating Q, values in [0,1] correspond to updating Q
window_s = [1000; 10000];
conf_interval = false;
%-----------------%

index_middle = sum(ff); % gives the number of iterations done with forgetting factor

if ~isempty(alpha_s)
    index_alpha = zeros(length(alpha_s), 1);
    for i = 1:length(alpha_s)
        [~, index_alpha(i)] = min(abs(value(1:index_middle) - alpha_s(i)));
    end
end

if ~isempty(window_s)
    index_window = zeros(length(window_s), 1);
    for i = 1:length(window_s)
        [~, index_window(i)] = min(abs(value(index_middle+1:end) - window_s(i)));
    end
    index_window = index_window + index_middle;
end

if ~exist('index_alpha')
    index_alpha = [];
end
if ~exist('index_window')
   index_window = [];
end

index_plot = [index_alpha; index_window];
n_plot = length(index_plot);

optitrack_euler = zeros(length(time), 3);

for i = 1:length(time)
    optitrack_euler(i,:) = quatToEuler(optitrack(i,:));
end

figure
subplot(4,1,1)
hold on
title('MEKF - Quaternion')
plot(time, optitrack(:, 1))
for j = 1:n_plot
    plot(time, kalman_quaternion{index_plot(j)}(:, 1))
end
hold off
grid
ylabel('$q_1$','Interpreter','latex','fontsize',12.0)
subplot(4,1,2)
hold on
plot(time, optitrack(:, 2))
for j = 1:n_plot
    plot(time, kalman_quaternion{index_plot(j)}(:, 2))
end
hold off
grid
ylabel('$q_2$','Interpreter','latex','fontsize',12.0)
subplot(4,1,3)
hold on
plot(time, optitrack(:, 3))
for j = 1:n_plot
    plot(time, kalman_quaternion{index_plot(j)}(:, 3))
end
hold off
grid
ylabel('$q_3$','Interpreter','latex','fontsize',12.0)
subplot(4,1,4)
hold on
plot(time, optitrack(:, 4))
for j = 1:n_plot
    plot(time, kalman_quaternion{index_plot(j)}(:, 4))
end
hold off
ylabel('$q_4$','Interpreter','latex','fontsize',12.0)
grid
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
a = '';
b = '';
if ~isempty(alpha_s)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_s(mode)), 1:length(alpha_s), 'UniformOutput', false);
end
if ~isempty(window_s)
    b = arrayfun(@(mode) sprintf('MEKF window = %d', window_s(mode)), 1:length(window_s), 'UniformOutput', false);
end
legend(['Optitrack', a, b])
saveas(gcf, 'figs/quatselect', 'png');


figure
subplot(3,1,1)
title('Estimated Euler angles')
hold on
plot(time, optitrack_euler(:, 1))
for j = 1:n_plot
    plot(time, MEKF_euler{index_plot(j)}(:, 1))
end
hold off
ylim([-0.5;0.5])
grid
ylabel('$\phi [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
plot(time, optitrack_euler(:, 2))
for j = 1:n_plot
    plot(time, MEKF_euler{index_plot(j)}(:, 2))
end
hold off
ylim([-0.5;0.5])
grid
ylabel('$\theta [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
plot(time, optitrack_euler(:, 3))
for j = 1:n_plot
    plot(time, MEKF_euler{index_plot(j)}(:, 3))
end
hold off
grid
ylabel('$\psi [rad]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
a = '';
b = '';
if ~isempty(alpha_s)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_s(mode)), 1:length(alpha_s), 'UniformOutput', false);
end
if ~isempty(window_s)
    b = arrayfun(@(mode) sprintf('MEKF window = %d', window_s(mode)), 1:length(window_s), 'UniformOutput', false);
end
legend(['Optitrack', a, b])
saveas(gcf, 'figs/eulerselect', 'png');


figure
subplot(3,1,1)
title('Estimation error')
hold on
for j = 1:n_plot
    plot(time, MEKF_euler_e{index_plot(j)}(:,1))
end
if conf_interval
    for j = 1:n_plot
        plot(time, 3*kalman_sigma{index_plot(j)}(:,1),'--')
        plot(time, -3*kalman_sigma{index_plot(j)}(:,1),'--')
    end
end
hold off
ylim([-0.5;0.5])
grid
ylabel('$\delta\phi [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
for j = 1:n_plot
    plot(time, MEKF_euler_e{index_plot(j)}(:,2))
end
if conf_interval
    for j = 1:n_plot
        plot(time, 3*kalman_sigma{index_plot(j)}(:,2),'--')
        plot(time, -3*kalman_sigma{index_plot(j)}(:,2),'--')
    end
end
hold off
ylim([-0.5;0.5])
grid
ylabel('$\delta\theta [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
for j = 1:n_plot
    plot(time, MEKF_euler_e{index_plot(j)}(:,3))
end
if conf_interval
    for j = 1:n_plot
        plot(time, 3*kalman_sigma{index_plot(j)}(:,3),'--')
        plot(time, -3*kalman_sigma{index_plot(j)}(:,3),'--')
    end
end
hold off
grid
ylabel('$\delta\psi [rad]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
a = '';
b = '';
if ~isempty(alpha_s)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f vs Optitrack', alpha_s(mode)), 1:length(alpha_s), 'UniformOutput', false);
end
if ~isempty(window_s)
    b = arrayfun(@(mode) sprintf('MEKF window = %d vs Optitrack', window_s(mode)), 1:length(window_s), 'UniformOutput', false);
end
legend([a, b])
saveas(gcf, 'figs/euler_eselect', 'png');


    % Using pwelch

fs = 1/(time(2)-time(1));
P_error = cell(n_plot, 3); % Each line contains each axis error for each value of alpha/window
f_error = cell(n_plot, 3);

na = 5;

for j = 1:n_plot
    for i = 1:3
       [P_error{j,i}, f_error{j,i}, ~] = pwelchrun(MEKF_euler_e{index_plot(j)}(:,i), na, fs); 
    end
end


figure
subplot(3,1,1)
title('PSD of estimation error')
hold on
set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')
for j = 1:n_plot
    plot(f_error{j,1}, P_error{j,1})
end
hold off
grid
ylabel('$\delta\phi [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')
for j = 1:n_plot
    plot(f_error{j,2}, P_error{j,2})
end
hold off
grid
ylabel('$\delta\theta [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')
for j = 1:n_plot
    plot(f_error{j,3}, P_error{j,3})
end
hold off
grid
ylabel('$\delta\psi [rad]$','Interpreter','latex','fontsize',12.0)
xlabel('$Frequency \ [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
a = '';
b = '';
if ~isempty(alpha_s)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f vs Optitrack', alpha_s(mode)), 1:length(alpha_s), 'UniformOutput', false);
end
if ~isempty(window_s)
    b = arrayfun(@(mode) sprintf('MEKF window = %d vs Optitrack', window_s(mode)), 1:length(window_s), 'UniformOutput', false);
end
legend([a, b])
saveas(gcf, 'figs/euler_e_psdselect', 'png');



figure
subplot(3,1,1)
title('Angular velocity')
hold on
for j = 1:n_plot
    plot(time, kalman_omega{index_plot(j)}(:,1))
end
plot(time, omega(:,1))
hold off
grid
ylabel('$p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
for j = 1:n_plot
    plot(time, kalman_omega{index_plot(j)}(:,2))
end
plot(time, omega(:,2))
hold off
grid
ylabel('$q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
for j = 1:n_plot
    plot(time, kalman_omega{index_plot(j)}(:,3))
end
plot(time, omega(:,3))
hold off
grid
ylabel('$r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
a = '';
b = '';
if ~isempty(alpha_s)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_s(mode)), 1:length(alpha_s), 'UniformOutput', false);
end
if ~isempty(window_s)
    b = arrayfun(@(mode) sprintf('MEKF window = %d', window_s(mode)), 1:length(window_s), 'UniformOutput', false);
end
legend([a, b, 'OB'])
saveas(gcf, 'figs/wselect', 'png');


figure
subplot(3,1,1)
title('Estimated bias')
hold on
for j = 1:n_plot
    plot(time, kalman_bias{index_plot(j)}(:,1))
end
hold off
grid
ylabel('$p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
for j = 1:n_plot
    plot(time, kalman_bias{index_plot(j)}(:,2))
end
hold off
grid
ylabel('$q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
for j = 1:n_plot
    plot(time, kalman_bias{index_plot(j)}(:,3))
end
hold off
grid
ylabel('$r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
a = '';
b = '';
if ~isempty(alpha_s)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_s(mode)), 1:length(alpha_s), 'UniformOutput', false);
end
if ~isempty(window_s)
    b = arrayfun(@(mode) sprintf('MEKF window = %d', window_s(mode)), 1:length(window_s), 'UniformOutput', false);
end
legend([a, b])
saveas(gcf, 'figs/biasselect', 'png');


figure
subplot(3,1,1)
title('Error angular velocities')
hold on;
for j = 1:n_plot
    plot(time, kalman_omega{index_plot(j)}(:,1) - omega(:,1))
end
hold off;
grid
ylabel('$\mathrm{error} \; p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on;
for j = 1:n_plot
    plot(time, kalman_omega{index_plot(j)}(:,2) - omega(:,2))
end
hold off;
grid;
ylabel('$\mathrm{error} \; q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on;
for j = 1:n_plot
    plot(time, kalman_omega{index_plot(j)}(:,3) - omega(:,3))
end
hold off;
grid
ylabel('$\mathrm{error} \; r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
a = '';
b = '';
if ~isempty(alpha_s)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f vs OB', alpha_s(mode)), 1:length(alpha_s), 'UniformOutput', false);
end
if ~isempty(window_s)
    b = arrayfun(@(mode) sprintf('MEKF window = %d vs OB', window_s(mode)), 1:length(window_s), 'UniformOutput', false);
end
legend([a, b])
saveas(gcf, 'figs/w_eselect', 'png');



%% Q against alpha analysis
clear; % NECESSARY

if ~exist('results') || ~exist('time') || ~exist('kalman_Q')
    get_data;
end

% Plot trace of Q over time for values of alpha

% INPUTS -------- %
alpha_Q = [0.7];  % -1 corresponds to not updating Q, values in [0,1] correspond to updating Q
window_Q = [1000; 10000];
%-----------------%

% Obtain reference data and eliminate from arrays
Q_ref = kalman_Q{1};
Q_ref = reshape(Q_ref(1, :, :), size(Q_ref, 2), size(Q_ref, 3));
trace_ref = trace(Q_ref);
npoints = npoints - 1;
value(1) = [];
kalman_Q(1) = [];
ff(1) = [];

index_middle = sum(ff); % gives the number of iterations done with forgetting factor

if ~isempty(alpha_Q)
    index_alpha = zeros(length(alpha_Q), 1);
    for i = 1:length(alpha_Q)
        [~, index_alpha(i)] = min(abs(value(1:index_middle) - alpha_Q(i)));
    end
end

if ~isempty(window_Q)
    index_window = zeros(length(window_Q), 1);
    for i = 1:length(window_Q)
        [~, index_window(i)] = min(abs(value(index_middle+1:end) - window_Q(i)));
    end
    index_window = index_window + index_middle;
end

index_Q = [index_alpha; index_window];
ndata = length(time);


figure; hold on; grid on;
title('Trace of Q matrix for several values of \alpha and window size');
set(gca, 'YScale', 'log');
xlabel('Time [$s$]', 'Interpreter', 'latex', 'fontsize', 12.0)
ylabel('Trace(Q)')
trace_index = zeros(ndata, length(index_Q));

for j = 1:length(index_Q)
   
    Q = kalman_Q{index_Q(j)};
    
    for i = 1:ndata
        Q_i = reshape(Q(i, :, :), size(Q, 2), size(Q, 3));
        trace_index(i, j) = trace(Q_i);
    end

    plot(time, trace_index(:, j));
%     if j <= length(alpha_Q)
%         ylabel(['Trace of Q for \alpha = ' num2str(alpha_Q(j))]);
%     else
%         ylabel(['Trace of Q for window = ' num2str(window_Q(j-length(alpha_Q)))])
%     end
end
plot(time, trace_ref*ones(length(time),1),'r--')
if ~isempty(alpha_Q)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_Q(mode)), 1:length(alpha_Q), 'UniformOutput', false);
end
if ~isempty(window_Q)
    b = arrayfun(@(mode) sprintf('MEKF window = %d', window_Q(mode)), 1:length(window_Q), 'UniformOutput', false);
end
legend([a, b],'Reference (constant Q)')


% Plot mean and variance of Q trace for each alpha
mean_Q = zeros(npoints, 1);
var_Q = zeros(npoints, 1);

for j = 1:npoints 
    Q = kalman_Q{j};
    trace_Q = zeros(ndata, 1);
    for i = 1:ndata
        Q_i = reshape(Q(i, :, :), size(Q, 2), size(Q, 3));
        trace_Q(i) = trace(Q_i);
    end
    mean_Q(j) = mean(trace_Q);
    var_Q(j) = var(trace_Q);
end

% Eliminate the results corresponding to alpha = 1
value_plot = value([1:index_middle-1 , index_middle+1:end]);
mean_plot = mean_Q([1:index_middle-1 , index_middle+1:end]);
var_plot = var_Q([1:index_middle-1 , index_middle+1:end]);

figure; 
subplot(2,1,1); hold on; grid on;
title('Mean and variance of trace(Q) for each alpha and window size')
plot(value_plot, mean_plot);
plot(value_plot, trace_ref*ones(length(value) - 1,1), 'r--');
set(gca, 'XScale', 'log');
set(gca, 'YScale', 'log');
xlabel('Alpha/Window size');
ylabel('Mean value of the trace of Q');

subplot(2,1,2); hold on; grid on;
plot(value_plot,var_plot);
set(gca, 'XScale', 'log');
set(gca, 'YScale', 'log');
xlabel('Alpha/Window size');
ylabel('Variance of the trace of Q');


% Computing frequency content of the trace of Q (one-sided spectrum)

    % Using fft
y = zeros(ndata/2 + 1, length(index_Q));

for i = 1:length(index_Q)
    aux = abs(fft(trace_index(:,i))/ndata); %normalized dft
    aux = aux(1:ndata/2 + 1);               %compute one-sided
    aux(2:end-1) = 2*aux(2:end-1);
    y(:,i) = aux;
end

fs = 1/(time(2)-time(1));
f = fs*(0:ndata/2)/ndata;

figure; hold on; grid on;
title('Magnitude of normalized discrete fourier transform of the trace of Q over time')
set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')
xlabel('$f [rad/s]$', 'interpreter', 'latex');
ylabel('Spectral density')
for i = 1:length(index_Q)
    plot(f, y(:,i)) 
end

if ~isempty(alpha_Q)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_Q(mode)), 1:length(alpha_Q), 'UniformOutput', false);
end
if ~isempty(window_Q)
    b = arrayfun(@(mode) sprintf('MEKF window = %d', window_Q(mode)), 1:length(window_Q), 'UniformOutput', false);
end
legend([a, b])

    % Using pwelch
    
Pxx = cell(1, length(index_Q));
f = cell(1, length(index_Q));

na = 5;

for i = 1:length(index_Q)
   [Pxx{i}, f{i}, ~] = pwelchrun(trace_index(:,i), na, fs); 
end

figure; hold on; grid on;
title('Power spectral density of trace of Q over time')
set(gca, 'YScale', 'log')
set(gca, 'XScale', 'log')
xlabel('$f [rad/s]$', 'interpreter', 'latex');
ylabel('PSD')

for i = 1:length(index_Q)
    plot(f{i}, Pxx{i}) 
end

if ~isempty(alpha_Q)
    a = arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_Q(mode)), 1:length(alpha_Q), 'UniformOutput', false);
end
if ~isempty(window_Q)
    b = arrayfun(@(mode) sprintf('MEKF window = %d', window_Q(mode)), 1:length(window_Q), 'UniformOutput', false);
end
legend([a, b])



%% Quaternion error
clearvars -except results time
close all

if ~exist('results') || ~exist('time')
    load('results.mat');
end

ff = extractfield(results, 'ff');
if iscell(ff)
    ff = cell2mat(ff);
end
results2 = results(ff==true);

if length(results2)==1
    disp('There are no points to plot.');
end

na = length(results2);
nq = size(results2(1).MEKF_q_e, 2);

qe.mean = zeros(na, nq);
qe.cov = zeros(na, nq, nq);

% Rotate yaw axis 90 degrees
for i = 1:na
    for j = 1:size(results2(1).MEKF_q_e, 1)
        results2(i).MEKF_q_e(j,:) =  -quatProd([0;0;cos(-pi/4);sin(-pi/4)], results2(i).MEKF_q_e(j,:)');
    end
end

for i = 1:na
    qe.mean(i, :) = mean(results2(i).MEKF_q_e);
    qe.cov(i, :, :) = covariance(results2(i).MEKF_q_e);
end

true_mean = qe.mean(1,:);
true_cov = squeeze(qe.cov(1,:,:));

qe.mean = qe.mean(2:end,:);
qe.cov = qe.cov(2:end,:,:);

alpha = extractfield(results2, 'value');
alpha = alpha(2:end);

%[~, im] = min(vecnorm(qe.mean(:, 1:4)-[0,0,0,1], 2, 2));
[~, im] = min(1-dot(qe.mean, repmat([0,0,0,1], length(qe.mean), 1), 2));
[~, ic] = min(sum(sum(abs(qe.cov), 3), 2));

figure;
subplot(4, 1, 1);
hold on;
plot(alpha, qe.mean(:, 1));
plot(alpha(im), qe.mean(im,1), '*c');
plot(alpha, ones(size(alpha))*true_mean(1), '--k');
title('Mean of quaternion error against alpha');
ylabel('$\bar q_{1e}$', 'interpreter', 'latex');
grid;
subplot(4, 1, 2);
hold on;
plot(alpha, qe.mean(:, 2));
plot(alpha(im), qe.mean(im,2), '*c');
plot(alpha, ones(size(alpha))*true_mean(2), '--k');
ylabel('$\bar q_{2e}$', 'interpreter', 'latex');
grid;
subplot(4, 1, 3);
hold on;
plot(alpha, qe.mean(:, 3));
plot(alpha(im), qe.mean(im,3), '*c');
plot(alpha, ones(size(alpha))*true_mean(3), '--k');
ylabel('$\bar q_{3e}$', 'interpreter', 'latex');
grid;
subplot(4, 1, 4);
hold on;
plot(alpha, qe.mean(:, 4));
plot(alpha(im), qe.mean(im,4), '*c');
plot(alpha, ones(size(alpha))*true_mean(4), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\bar q_{4e}$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'estimated Q');

figure;
subplot(4, 1, 1);
hold on;
plot(alpha, qe.cov(:, 1, 1));
plot(alpha(ic), qe.cov(ic, 1, 1), '*c');
plot(alpha, ones(size(alpha))*true_cov(1,1), '--k');
title('Variance of quaternion error against alpha');
ylabel('$\mathrm{Var}(q_{1e})$', 'interpreter', 'latex');
grid;
subplot(4, 1, 2);
hold on;
plot(alpha, qe.cov(:, 2, 2));
plot(alpha(ic), qe.cov(ic, 2, 2), '*c');
plot(alpha, ones(size(alpha))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(q_{2e})$', 'interpreter', 'latex');
grid;
subplot(4, 1, 3);
hold on;
plot(alpha, qe.cov(:, 3, 3));
plot(alpha(ic), qe.cov(ic, 3, 3), '*c');
plot(alpha, ones(size(alpha))*true_cov(3,3), '--k');
ylabel('$\mathrm{Var}(q_{3e})$', 'interpreter', 'latex');
grid;
subplot(4, 1, 4);
hold on;
plot(alpha, qe.cov(:, 4, 4));
plot(alpha(ic), qe.cov(ic, 4, 4), '*c');
plot(alpha, ones(size(alpha))*true_cov(4,4), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\mathrm{Var}(q_{4e})$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'estimated Q');


%% Euler angles error against alpha
clearvars -except results time
close all

if ~exist('results') || ~exist('time')
    load('results.mat');
end

ff = extractfield(results, 'ff');
if iscell(ff)
    ff = cell2mat(ff);
end
results2 = results(ff==true);

if length(results2)==1
    disp('There are no points to plot.');
end

na = length(results2);
ne = size(results2(1).MEKF_euler_e, 2);

eul_a.mean = zeros(na, ne);
eul_a.cov = zeros(na, ne, ne);

% Rotate yaw axis 90 degrees
for i = 1:na
    results2(i).MEKF_euler_e =  results2(i).MEKF_euler_e - [0,0,pi/2];
end

for i = 1:na
    eul_a.mean(i, :) = abs(mean(results2(i).MEKF_euler_e));
    eul_a.cov(i, :, :) = covariance(results2(i).MEKF_euler_e);
end

true_mean = eul_a.mean(1,:);
true_cov = squeeze(eul_a.cov(1,:,:));

eul_a.mean = eul_a.mean(2:end,:);
eul_a.cov = eul_a.cov(2:end,:,:);

alpha = extractfield(results2, 'value');
alpha = alpha(2:end);

[~, im] = min(vecnorm(eul_a.mean(:, 1:3), 2, 2));
[~, ic] = min(sum(sum(abs(eul_a.cov), 3), 2));


figure;
subplot(3, 1, 1);
hold on;
plot(alpha, eul_a.mean(:, 1));
plot(alpha(im), eul_a.mean(im,1), '*c');
plot(alpha, ones(size(alpha))*true_mean(1), '--k');
title('Mean of euler angle error against alpha');
ylabel('$\bar \delta_\phi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 2);
hold on;
plot(alpha, eul_a.mean(:, 2));
plot(alpha(im), eul_a.mean(im,2), '*c');
plot(alpha, ones(size(alpha))*true_mean(2), '--k');
ylabel('$\bar \delta_\theta [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 3);
hold on;
plot(alpha, eul_a.mean(:, 3));
plot(alpha(im), eul_a.mean(im,3), '*c');
plot(alpha, ones(size(alpha))*true_mean(3), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\bar \delta_\psi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'estimated Q');

figure;
subplot(3, 1, 1);
hold on;
plot(alpha, eul_a.cov(:, 1, 1));
plot(alpha(ic), eul_a.cov(ic, 1, 1), '*c');
plot(alpha, ones(size(alpha))*true_cov(1,1), '--k');
title('Variance of euler angle error against alpha');
ylabel('$\mathrm{Var}(\delta_\phi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 2);
hold on;
plot(alpha, eul_a.cov(:, 2, 2));
plot(alpha(ic), eul_a.cov(ic, 2, 2), '*c');
plot(alpha, ones(size(alpha))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(\delta_\theta) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 3);
hold on;
plot(alpha, eul_a.cov(:, 3, 3));
plot(alpha(ic), eul_a.cov(ic, 3, 3), '*c');
plot(alpha, ones(size(alpha))*true_cov(3,3), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\mathrm{Var}(\delta_\psi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'estimated Q');


%% Angular rates error against alpha
clearvars -except results time omega
close all

if ~exist('results') || ~exist('time') || ~exist('omega')
    load('results.mat');
end

ff = extractfield(results, 'ff');
if iscell(ff)
    ff = cell2mat(ff);
end

results2 = results(ff==true);

if length(results2)==1
    disp('There are no points to plot.');
end

na = length(results2);
ne = size(results2(1).kalman_omega, 2);
l = sum(isnan(omega(:,1)));     % some of omega elements are NaN. they'll be filtered out

om_a.mean = zeros(na, ne);
om_a.cov = zeros(na, ne, ne);

ome = zeros(na, size(omega,1)-l, ne);

% Calculate error
for i = 1:na
    ome(i,:,:) =  results2(i).kalman_omega(l+1:end,:) - omega(l+1:end,:);
end

for i = 1:na
    om_a.mean(i, :) = abs(mean(ome(i,:,:)));
    om_a.cov(i, :, :) = covariance(reshape(ome(i,:,:), size(ome,2), size(ome,3)));
end

true_mean = om_a.mean(1,:);
true_cov = squeeze(om_a.cov(1,:,:));

om_a.mean = om_a.mean(2:end,:);
om_a.cov = om_a.cov(2:end,:,:);

alpha = extractfield(results2, 'value');
alpha = alpha(2:end);

[~, im] = min(vecnorm(om_a.mean(:, 1:2), 2, 2));
[~, ic] = min(sum(sum(abs(om_a.cov), 3), 2));

figure;
subplot(3, 1, 1);
hold on;
plot(alpha, om_a.mean(:, 1));
plot(alpha(im), om_a.mean(im,1), '*c');
plot(alpha, ones(size(alpha))*true_mean(1), '--k');
title('Mean of angular rates error against alpha');
ylabel('$\bar \delta_\phi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 2);
hold on;
plot(alpha, om_a.mean(:, 2));
plot(alpha(im), om_a.mean(im,2), '*c');
plot(alpha, ones(size(alpha))*true_mean(2), '--k');
ylabel('$\bar \delta_\theta [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 3);
hold on;
plot(alpha, om_a.mean(:, 3));
plot(alpha(im), om_a.mean(im,3), '*c');
plot(alpha, ones(size(alpha))*true_mean(3), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\bar \delta_\psi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'estimated Q');

figure;
subplot(3, 1, 1);
hold on;
plot(alpha, om_a.cov(:, 1, 1));
plot(alpha(ic), om_a.cov(ic, 1, 1), '*c');
plot(alpha, ones(size(alpha))*true_cov(1,1), '--k');
title('Variance of angular rates error against alpha');
ylabel('$\mathrm{Var}(\delta_\phi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 2);
hold on;
plot(alpha, om_a.cov(:, 2, 2));
plot(alpha(ic), om_a.cov(ic, 2, 2), '*c');
plot(alpha, ones(size(alpha))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(\delta_\theta) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 3);
hold on;
plot(alpha, om_a.cov(:, 3, 3));
plot(alpha(ic), om_a.cov(ic, 3, 3), '*c');
plot(alpha, ones(size(alpha))*true_cov(3,3), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\mathrm{Var}(\delta_\psi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'estimated Q');


%% Euler angles error against mean window
clearvars -except results time
close all

if ~exist('results') || ~exist('time')
    load('results.mat');
end

ff = extractfield(results, 'ff');
if iscell(ff)
    ff = cell2mat(ff);
end
value = extractfield(results, 'value');

results2 = results(or(value==-1, ff==false));

na = length(results2);
ne = size(results2(1).MEKF_euler_e, 2);

eul_w.mean = zeros(na, ne);
eul_w.cov = zeros(na, ne, ne);

% Rotate yaw axis 90 degrees
% for i = 1:na
%     results2(i).MEKF_euler_e =  results2(i).MEKF_euler_e - [0,0,pi/2];
% end

for i = 1:na
    eul_w.mean(i, :) = abs(mean(results2(i).MEKF_euler_e));
    eul_w.cov(i, :, :) = covariance(results2(i).MEKF_euler_e);
end

true_mean = eul_w.mean(1,:);
true_cov = squeeze(eul_w.cov(1,:,:));

eul_w.mean = eul_w.mean(2:end,:);
eul_w.cov = eul_w.cov(2:end,:,:);

value = extractfield(results2, 'value');
value = value(2:end);

[~, im] = min(vecnorm(eul_w.mean(:, 1:2), 2, 2));
[~, ic] = min(sum(sum(abs(eul_w.cov), 3), 2));

figure;
subplot(3, 1, 1);
hold on;
plot(value, eul_w.mean(:, 1));
plot(value(im), eul_w.mean(im,1), '*c');
plot(value, ones(size(value))*true_mean(1), '--k');
title('Mean of euler angle error against window size of mean');
ylabel('$\bar \delta_\phi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
subplot(3, 1, 2);
hold on;
plot(value, eul_w.mean(:, 2));
plot(value(im), eul_w.mean(im,2), '*c');
plot(value, ones(size(value))*true_mean(2), '--k');
ylabel('$\bar \delta_\theta [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
subplot(3, 1, 3);
hold on;
plot(value, eul_w.mean(:, 3));
plot(value(im), eul_w.mean(im,3), '*c');
plot(value, ones(size(value))*true_mean(3), '--k');
xlabel('window size [points]');
ylabel('$\bar \delta_\psi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
legend('varying window', 'minimum', 'estimated Q', 'location', 'southwest');

figure;
subplot(3, 1, 1);
hold on;
plot(value, eul_w.cov(:, 1, 1));
plot(value(ic), eul_w.cov(ic, 1, 1), '*c');
plot(value, ones(size(value))*true_cov(1,1), '--k');
title('Variance of euler angle error against window size of mean');
ylabel('$\mathrm{Var}(\delta_\phi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
subplot(3, 1, 2);
hold on;
plot(value, eul_w.cov(:, 2, 2));
plot(value(ic), eul_w.cov(ic, 2, 2), '*c');
plot(value, ones(size(value))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(\delta_\theta) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
subplot(3, 1, 3);
hold on; 
plot(value, eul_w.cov(:, 3, 3));
plot(value(ic), eul_w.cov(ic, 3, 3), '*c');
plot(value, ones(size(value))*true_cov(3,3), '--k');
xlabel('window size [points]');
ylabel('$\mathrm{Var}(\delta_\psi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
legend('varying window', 'minimum', 'estimated Q', 'location', 'northwest');


%% Angular rates error against mean window
clearvars -except results time omega
close all

if ~exist('results') || ~exist('time') || ~exist('omega')
    load('results.mat');
end

ff = extractfield(results, 'ff');
if iscell(ff)
    ff = cell2mat(ff);
end
value = extractfield(results, 'value');

results2 = results(or(value==-1, ff==false));

na = length(results2);
ne = size(results2(1).kalman_omega, 2);
l = sum(isnan(omega(:,1)));     % some of omega elements are NaN. they'll be filtered out

om_w.mean = zeros(na, ne);
om_w.cov = zeros(na, ne, ne);

ome = zeros(na, size(omega,1)-l, ne);

% Calculate error
for i = 1:na
    ome(i,:,:) =  results2(i).kalman_omega(l+1:end,:) - omega(l+1:end,:);
end

for i = 1:na
    om_w.mean(i, :) = abs(mean(ome(i,:,:)));
    om_w.cov(i, :, :) = covariance(reshape(ome(i,:,:), size(ome,2), size(ome,3)));
end

true_mean = om_w.mean(1,:);
true_cov = squeeze(om_w.cov(1,:,:));

om_w.mean = om_w.mean(2:end,:);
om_w.cov = om_w.cov(2:end,:,:);

value = extractfield(results2, 'value');
value = value(2:end);

[~, im] = min(vecnorm(om_w.mean(:, 1:2), 2, 2));
[~, ic] = min(sum(sum(abs(om_w.cov), 3), 2));

figure;
subplot(3, 1, 1);
hold on;
plot(value, om_w.mean(:, 1));
plot(value(im), om_w.mean(im,1), '*c');
plot(value, ones(size(value))*true_mean(1), '--k');
title('Mean of angular rate error against window size of mean');
ylabel('$\bar p_e [\mathrm{rad/s}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
subplot(3, 1, 2);
hold on;
plot(value, om_w.mean(:, 2));
plot(value(im), om_w.mean(im,2), '*c');
plot(value, ones(size(value))*true_mean(2), '--k');
ylabel('$\bar q_e [\mathrm{rad/s}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
subplot(3, 1, 3);
hold on;
plot(value, om_w.mean(:, 3));
plot(value(im), om_w.mean(im,3), '*c');
plot(value, ones(size(value))*true_mean(3), '--k');
xlabel('window size [points]');
ylabel('$\bar r_e [\mathrm{rad/s}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
legend('varying window', 'minimum', 'estimated Q', 'location', 'southwest');

figure;
subplot(3, 1, 1);
hold on;
plot(value, om_w.cov(:, 1, 1));
plot(value(ic), om_w.cov(ic, 1, 1), '*c');
plot(value, ones(size(value))*true_cov(1,1), '--k');
title('Variance of angular rate error against window size of mean');
ylabel('$\mathrm{Var}(p_e) [(\mathrm{rad/s})^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
subplot(3, 1, 2);
hold on;
plot(value, om_w.cov(:, 2, 2));
plot(value(ic), om_w.cov(ic, 2, 2), '*c');
plot(value, ones(size(value))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(q_e) [(\mathrm{rad/s})^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
subplot(3, 1, 3);
hold on; 
plot(value, om_w.cov(:, 3, 3));
plot(value(ic), om_w.cov(ic, 3, 3), '*c');
plot(value, ones(size(value))*true_cov(3,3), '--k');
xlabel('window size [points]');
ylabel('$\mathrm{Var}(r_e) [(\mathrm{rad/s})^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
xlim([value(1), value(end)]);
legend('varying window', 'minimum', 'estimated Q', 'location', 'northwest');


%% Functions
function C = covariance(X)
    % Get time dimension in first position
    if size(X, 1)<size(X, 2)
        X = X';
    end
    
    C = zeros(size(X, 2));
    
    for i = 1:size(C, 1)
        for j = i:size(C, 2)
            if i==j
                C(i,j) = var(X(:, i));
            else
                C(i,j) = covxy(X(:,i), X(:,j));
            end
        end
    end
    
    % Mirror covariance matrix, since it is symmetric
    C = triu(C) + triu(C,1)';
    
    function c = covxy(x, y)
        c = (x-mean(x))'*(y-mean(y))/(length(x)-1);
    end
end

