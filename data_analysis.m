% Obtain data in double array for alpha vals and cell arrays for the rest from results.mat
get_data;


%% Plot

if ~exist('results') || ~exist('time')
    get_data;
end

alpha_show = [1; 0.7];  % -1 corresponds to not updating Q, values in [0,1] correspond to updating Q
n_show = length(alpha_show);
index_show = zeros(length(alpha_show), 1);

for i = 1:length(index_show)
    [~, index_show(i)] = min(abs(alpha - alpha_show(i)));
end

optitrack_euler = zeros(length(time), 3);

for i = 1:length(time)
    optitrack_euler(i,:) = quatToEuler(optitrack(i,:));
end


figure
subplot(4,1,1)
hold on
title('MEKF - Quaternion')
plot(time, optitrack(:, 1))
for j = 1:n_show
    plot(time, kalman_quaternion{index_show(j)}(:, 1))
end
hold off
grid
ylabel('$q_1$','Interpreter','latex','fontsize',12.0)
subplot(4,1,2)
hold on
plot(time, optitrack(:, 2))
for j = 1:n_show
    plot(time, kalman_quaternion{index_show(j)}(:, 2))
end
hold off
grid
ylabel('$q_2$','Interpreter','latex','fontsize',12.0)
subplot(4,1,3)
hold on
plot(time, optitrack(:, 3))
for j = 1:n_show
    plot(time, kalman_quaternion{index_show(j)}(:, 3))
end
hold off
grid
ylabel('$q_3$','Interpreter','latex','fontsize',12.0)
subplot(4,1,4)
hold on
plot(time, optitrack(:, 4))
for j = 1:n_show
    plot(time, kalman_quaternion{index_show(j)}(:, 4))
end
hold off
ylabel('$q_4$','Interpreter','latex','fontsize',12.0)
grid
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend(['Optitrack',arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_show(mode)), 1:n_show, 'UniformOutput', false)])


figure
subplot(3,1,1)
title('Estimated Euler angles')
hold on
plot(time, optitrack_euler(:, 1))
for j = 1:n_show
    plot(time, MEKF_euler{index_show(j)}(:, 1))
end
hold off
ylim([-0.5;0.5])
grid
ylabel('$\phi [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
plot(time, optitrack_euler(:, 2))
for j = 1:n_show
    plot(time, MEKF_euler{index_show(j)}(:, 2))
end
hold off
ylim([-0.5;0.5])
grid
ylabel('$\theta [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
plot(time, optitrack_euler(:, 3))
for j = 1:n_show
    plot(time, MEKF_euler{index_show(j)}(:, 3))
end
hold off
grid
ylabel('$\psi [rad]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend(['Optitrack',arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_show(mode)), 1:n_show, 'UniformOutput', false)])

figure
subplot(3,1,1)
title('Estimation error')
hold on
for j = 1:n_show
    plot(time, MEKF_euler_e{index_show(j)}(:,1))
    plot(time, 3*kalman_sigma{index_show(j)}(:,1),'--')
    plot(time, -3*kalman_sigma{index_show(j)}(:,1),'--')
end
% plot(time, MEKF1_euler_e(:,1))
% plot(time, MEKF2_euler_e(:,1))
% plot(time, 3*kalman_sigma1(:,1),'r--')
% plot(time, -3*kalman_sigma1(:,1),'r--')
% plot(time, 3*kalman_sigma2(:,1),'b--')
% plot(time, -3*kalman_sigma2(:,1),'b--')
hold off
ylim([-0.5;0.5])
grid
ylabel('$\delta\phi [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
for j = 1:n_show
    plot(time, MEKF_euler_e{index_show(j)}(:,1))
    plot(time, 3*kalman_sigma{index_show(j)}(:,1),'--')
    plot(time, -3*kalman_sigma{index_show(j)}(:,1),'--')
end
% plot(time, MEKF1_euler_e(:,2))
% plot(time, MEKF2_euler_e(:,2))
% plot(time, 3*kalman_sigma1(:,2),'r--')
% plot(time, -3*kalman_sigma1(:,2),'r--')
% plot(time, 3*kalman_sigma2(:,2),'b--')
% plot(time, -3*kalman_sigma2(:,2),'b--')
hold off
ylim([-0.5;0.5])
grid
ylabel('$\delta\theta [rad]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
for j = 1:n_show
    plot(time, MEKF_euler_e{index_show(j)}(:,1))
    plot(time, 3*kalman_sigma{index_show(j)}(:,1),'--')
    plot(time, -3*kalman_sigma{index_show(j)}(:,1),'--')
end
% plot(time, MEKF1_euler_e(:,3))
% plot(time, MEKF2_euler_e(:,3))
% plot(time, 3*kalman_sigma1(:,3),'r--')
% plot(time, -3*kalman_sigma1(:,3),'r--')
% plot(time, 3*kalman_sigma2(:,3),'b--')
% plot(time, -3*kalman_sigma2(:,3),'b--')
hold off
grid
ylabel('$\delta\psi [rad]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend(arrayfun(@(mode) sprintf('MEKF alpha = %.2f vs Optitrack', alpha_show(mode)), 1:n_show, 'UniformOutput', false))

figure
subplot(3,1,1)
title('Angular velocity')
hold on
for j = 1:n_show
    plot(time, kalman_omega{index_show(j)}(:,1))
end
plot(time, omega(:,1))
hold off
grid
ylabel('$p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
for j = 1:n_show
    plot(time, kalman_omega{index_show(j)}(:,2))
end
plot(time, omega(:,2))
hold off
grid
ylabel('$q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
for j = 1:n_show
    plot(time, kalman_omega{index_show(j)}(:,3))
end
plot(time, omega(:,3))
hold off
grid
ylabel('$r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend([arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_show(mode)), 1:n_show, 'UniformOutput', false),'OB'])

figure
subplot(3,1,1)
title('Estimated bias')
hold on
for j = 1:n_show
    plot(time, kalman_bias{index_show(j)}(:,1))
end
hold off
grid
ylabel('$p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on
for j = 1:n_show
    plot(time, kalman_bias{index_show(j)}(:,2))
end
hold off
grid
ylabel('$q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on
for j = 1:n_show
    plot(time, kalman_bias{index_show(j)}(:,3))
end
hold off
grid
ylabel('$r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend(arrayfun(@(mode) sprintf('MEKF alpha = %.2f', alpha_show(mode)), 1:n_show, 'UniformOutput', false))

figure
subplot(3,1,1)
title('Error angular velocities')
hold on;
for j = 1:n_show
    plot(time, kalman_bias{index_show(j)}(:,1) - omega(:,1))
end
hold off;
grid
ylabel('$\mathrm{error} \; p [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,2)
hold on;
for j = 1:n_show
    plot(time, kalman_bias{index_show(j)}(:,2) - omega(:,2))
end
hold off;
grid;
ylabel('$\mathrm{error} \; q [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
subplot(3,1,3)
hold on;
for j = 1:n_show
    plot(time, kalman_bias{index_show(j)}(:,3) - omega(:,3))
end
hold off;
grid
ylabel('$\mathrm{error} \; r [\frac{rad}{s}]$','Interpreter','latex','fontsize',12.0)
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
legend(arrayfun(@(mode) sprintf('MEKF alpha = %.2f vs OB', alpha_show(mode)), 1:n_show, 'UniformOutput', false))


%% Q against alpha analysis

if ~exist('results') || ~exist('time') || ~exist('kalman_Q')
    get_data;
end

if alpha(1) == -1
    Q_ref = reshape(kalman_Q{1}(1, :, :), size(kalman_Q{1}, 2), size(kalman_Q{1}, 3));
    trace_ref = trace(Q_ref);
    kalman_Q(1) = [];
    npoints = npoints - 1;
    alpha = alpha(2:end);
end

% Plot trace of Q over time for values of alpha
alpha_vec = [0, 0.25, 0.5, 0.75, 1];
n_alpha = length(alpha_vec);

index_Q = zeros(1,n_alpha);
ndata = length(time);

for i = 1:n_alpha
    [~,index_Q(i)] = min(abs(alpha - alpha_vec(i)));
end

figure; hold on; grid on;
title('Trace of Q matrix for several values of \alpha');

for j = 1:n_alpha
   
    Q = kalman_Q{index_Q(j)};
    trace_Q = zeros(ndata, 1);
    eig_Q = zeros(ndata, 6);
    
    for i = 1:ndata
        Q_i = reshape(Q(i, :, :), size(Q, 2), size(Q, 3));
        trace_Q(i) = trace(Q_i);
        eig_Q(i, :) = eig(Q_i);
    end
    
    subplot(n_alpha, 1, j); hold on;
    plot(time, trace_Q);
    plot(time, trace_ref*ones(length(time),1),'r--')
    set(gca, 'YScale', 'log');
    xlabel('Time [$s$]', 'Interpreter', 'latex', 'fontsize', 12.0)
    ylabel(['Trace of Q for \alpha = ' num2str(alpha_vec(j))]);
end


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

figure; 
subplot(2,1,1); hold on; grid on;
title('Mean and variance of trace(Q) for each alpha')
plot(alpha, mean_Q);
plot(alpha, trace_ref*ones(length(alpha),1), 'r--');
set(gca, 'YScale', 'log');
xlabel('Alpha');
ylabel('Mean value of the trace of Q');
subplot(2,1,2); hold on; grid on;
plot(alpha,var_Q);
xlabel('Alpha');
ylabel('Variance of the trace of Q');


%% Quaternion error
close all

if ~exist('results') || ~exist('time')
    load('results.mat');
end

ff = extractfield(results, 'ff');
results = results(ff==true);

na = length(results);
nq = size(results(1).MEKF_q_e, 2);

qe.mean = zeros(na, nq);
qe.cov = zeros(na, nq, nq);

for i = 1:na
    qe.mean(i, :) = mean(results(i).MEKF_q_e);
    qe.cov(i, :, :) = covariance(results(i).MEKF_q_e);
end

true_mean = qe.mean(1,:);
true_cov = squeeze(qe.cov(1,:,:));
no_appr_mean = qe.mean(2,:);
no_appr_cov = squeeze(qe.cov(2,:,:));
qe.mean = qe.mean(3:end,:);
qe.cov = qe.cov(3:end,:,:);

alpha = extractfield(results, 'value');
alpha = alpha(3:end);

%[~, im] = min(vecnorm(qe.mean(:, 1:4)-[0,0,0,1], 2, 2));
[~, im] = min(1-dot(qe.mean, repmat([0,0,0,1], length(qe.mean), 1), 2));
[~, ic] = min(sum(sum(abs(qe.cov), 3), 2));

figure;
subplot(4, 1, 1);
hold on;
plot(alpha, qe.mean(:, 1));
plot(alpha(im), qe.mean(im,1), '*c');
plot(alpha, ones(size(alpha))*true_mean(1), '--k');
plot(alpha, ones(size(alpha))*no_appr_mean(1), '--r');
title('Mean of quaternion error against alpha');
ylabel('$\bar q_{1e}$', 'interpreter', 'latex');
grid;
subplot(4, 1, 2);
hold on;
plot(alpha, qe.mean(:, 2));
plot(alpha(im), qe.mean(im,2), '*c');
plot(alpha, ones(size(alpha))*true_mean(2), '--k');
plot(alpha, ones(size(alpha))*no_appr_mean(2), '--r');
ylabel('$\bar q_{2e}$', 'interpreter', 'latex');
grid;
subplot(4, 1, 3);
hold on;
plot(alpha, qe.mean(:, 3));
plot(alpha(im), qe.mean(im,3), '*c');
plot(alpha, ones(size(alpha))*true_mean(3), '--k');
plot(alpha, ones(size(alpha))*no_appr_mean(3), '--r');
ylabel('$\bar q_{3e}$', 'interpreter', 'latex');
grid;
subplot(4, 1, 4);
hold on;
plot(alpha, qe.mean(:, 4));
plot(alpha(im), qe.mean(im,4), '*c');
plot(alpha, ones(size(alpha))*true_mean(4), '--k');
plot(alpha, ones(size(alpha))*no_appr_mean(4), '--r');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\bar q_{4e}$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'real Q', 'no approx alpha');

figure;
subplot(4, 1, 1);
hold on;
plot(alpha, qe.cov(:, 1, 1));
plot(alpha(ic), qe.cov(ic, 1, 1), '*c');
plot(alpha, ones(size(alpha))*true_cov(1,1), '--k');
plot(alpha, ones(size(alpha))*no_appr_cov(1,1), '--r');
title('Variance of quaternion error against alpha');
ylabel('$\mathrm{Var}(q_{1e})$', 'interpreter', 'latex');
grid;
subplot(4, 1, 2);
hold on;
plot(alpha, qe.cov(:, 2, 2));
plot(alpha(ic), qe.cov(ic, 2, 2), '*c');
plot(alpha, ones(size(alpha))*true_cov(2,2), '--k');
plot(alpha, ones(size(alpha))*no_appr_cov(2,2), '--r');
ylabel('$\mathrm{Var}(q_{2e})$', 'interpreter', 'latex');
grid;
subplot(4, 1, 3);
hold on;
plot(alpha, qe.cov(:, 3, 3));
plot(alpha(ic), qe.cov(ic, 3, 3), '*c');
plot(alpha, ones(size(alpha))*true_cov(3,3), '--k');
plot(alpha, ones(size(alpha))*no_appr_cov(3,3), '--r');
ylabel('$\mathrm{Var}(q_{3e})$', 'interpreter', 'latex');
grid;
subplot(4, 1, 4);
hold on;
plot(alpha, qe.cov(:, 4, 4));
plot(alpha(ic), qe.cov(ic, 4, 4), '*c');
plot(alpha, ones(size(alpha))*true_cov(4,4), '--k');
plot(alpha, ones(size(alpha))*no_appr_cov(4,4), '--r');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\mathrm{Var}(q_{4e})$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'real Q', 'no approx alpha');


%% Euler angles error against alpha
close all

if ~exist('results') || ~exist('time')
    load('results.mat');
end

ff = extractfield(results, 'ff');
results = results(ff==true);

if length(results)==1 && results(1).value==-1
    disp('There are no points to plot.');
end

na = length(results);
ne = size(results(1).MEKF_euler_e, 2);

eul.mean = zeros(na, ne);
eul.cov = zeros(na, ne, ne);

for i = 1:na
    eul.mean(i, :) = mean(results(i).MEKF_euler_e);
    eul.cov(i, :, :) = covariance(results(i).MEKF_euler_e);
end

true_mean = eul.mean(1,:);
true_cov = squeeze(eul.cov(1,:,:));

eul.mean = eul.mean(2:end,:);
eul.cov = eul.cov(2:end,:,:);

alpha = extractfield(results, 'value');
alpha = alpha(2:end);

[~, im] = min(vecnorm(eul.mean(:, 1:3), 2, 2));
[~, ic] = min(sum(sum(abs(eul.cov), 3), 2));


figure;
subplot(3, 1, 1);
hold on;
plot(alpha, eul.mean(:, 1));
plot(alpha(im), eul.mean(im,1), '*c');
plot(alpha, ones(size(alpha))*true_mean(1), '--k');
title('Mean of euler angle error against alpha');
ylabel('$\bar \delta_\phi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 2);
hold on;
plot(alpha, eul.mean(:, 2));
plot(alpha(im), eul.mean(im,2), '*c');
plot(alpha, ones(size(alpha))*true_mean(2), '--k');
ylabel('$\bar \delta_\theta [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 3);
hold on;
plot(alpha, eul.mean(:, 3));
plot(alpha(im), eul.mean(im,3), '*c');
plot(alpha, ones(size(alpha))*true_mean(3), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\bar \delta_\psi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'real Q');

figure;
subplot(3, 1, 1);
hold on;
plot(alpha, eul.cov(:, 1, 1));
plot(alpha(ic), eul.cov(ic, 1, 1), '*c');
plot(alpha, ones(size(alpha))*true_cov(1,1), '--k');
title('Variance of euler angle error against alpha');
ylabel('$\mathrm{Var}(\delta_\phi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 2);
hold on;
plot(alpha, eul.cov(:, 2, 2));
plot(alpha(ic), eul.cov(ic, 2, 2), '*c');
plot(alpha, ones(size(alpha))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(\delta_\theta) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 3);
hold on;
plot(alpha, eul.cov(:, 3, 3));
plot(alpha(ic), eul.cov(ic, 3, 3), '*c');
plot(alpha, ones(size(alpha))*true_cov(3,3), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\mathrm{Var}(\delta_\psi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
legend('varying alpha', 'minimum', 'real Q');


%% Euler angles error against mean window
close all

if ~exist('results') || ~exist('time')
    load('results.mat');
end

ff = extractfield(results, 'ff')';
value = extractfield(results, 'value')';

results = results(value==-1+ff==false);

na = length(results);
ne = size(results(1).MEKF_euler_e, 2);

eul.mean = zeros(na, ne);
eul.cov = zeros(na, ne, ne);

for i = 1:na
    eul.mean(i, :) = abs(mean(results(i).MEKF_euler_e));
    eul.cov(i, :, :) = covariance(results(i).MEKF_euler_e);
end

true_mean = eul.mean(1,:);
true_cov = squeeze(eul.cov(1,:,:));

eul.mean = eul.mean(2:end,:);
eul.cov = eul.cov(2:end,:,:);

value = value(2:end);

[~, im] = min(vecnorm(eul.mean(:, 1:3), 2, 2));
[~, ic] = min(sum(sum(abs(eul.cov), 3), 2));

figure;
subplot(3, 1, 1);
hold on;
plot(value, eul.mean(:, 1));
plot(value(im), eul.mean(im,1), '*c');
plot(value, ones(size(value))*true_mean(1), '--k');
title('Mean of euler angle error against window size of mean');
ylabel('$\bar \delta_\phi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
subplot(3, 1, 2);
hold on;
plot(value, eul.mean(:, 2));
plot(value(im), eul.mean(im,2), '*c');
plot(value, ones(size(value))*true_mean(2), '--k');
ylabel('$\bar \delta_\theta [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
subplot(3, 1, 3);
hold on;
plot(value, eul.mean(:, 3));
plot(value(im), eul.mean(im,3), '*c');
plot(value, ones(size(value))*true_mean(3), '--k');
xlabel('window size [points]');
ylabel('$\bar \delta_\psi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
legend('varying window', 'minimum', 'real Q');

figure;
subplot(3, 1, 1);
hold on;
plot(value, eul.cov(:, 1, 1));
plot(value(ic), eul.cov(ic, 1, 1), '*c');
plot(value, ones(size(value))*true_cov(1,1), '--k');
title('Variance of euler angle error against window size of mean');
ylabel('$\mathrm{Var}(\delta_\phi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
subplot(3, 1, 2);
hold on;
plot(value, eul.cov(:, 2, 2));
plot(value(ic), eul.cov(ic, 2, 2), '*c');
plot(value, ones(size(value))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(\delta_\theta) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
subplot(3, 1, 3);
hold on;
plot(value, eul.cov(:, 3, 3));
plot(value(ic), eul.cov(ic, 3, 3), '*c');
plot(value, ones(size(value))*true_cov(3,3), '--k');
xlabel('window size [points]');
ylabel('$\mathrm{Var}(\delta_\psi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
set(gca, 'XScale', 'log');
legend('varying window', 'minimum', 'real Q');


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

