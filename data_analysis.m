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
hold off;


%% Q update effect analysis

ndata = length(time);

trace_Q = zeros(ndata,1);
eig_Q = zeros(ndata,6);

for i = 1:ndata
   Q = reshape(kalman_Q2(i,:,:), size(kalman_Q2,2), size(kalman_Q2,3));
   trace_Q(i) = trace(Q);
   eig_Q(i,:) = eig(Q);
end

figure; hold on; grid on;
plot(time, trace_Q);
title('Trace of Q matrix over time for alpha =');
xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
ylabel('$\mathrm{Trace \ of \ Q}$','Interpreter','latex','fontsize',12.0)


%% Quaternion error
close all

if ~exist('results') || ~exist('time')
    load('results.mat');
end

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
qe.mean = qe.mean(2:end,:);
qe.cov = qe.cov(2:end,:,:);

alpha = extractfield(results, 'alpha');
alpha = alpha(2:end);

%[~, im] = min(vecnorm(qe.mean(:, 1:4)-[0,0,0,1], 2, 2));
[~, im] = min(1-dot(qe.mean, repmat([0,0,0,1], length(qe.mean), 1), 2));
[~, ic] = min(sum(sum(abs(qe.cov), 3), 2));

figure;
subplot(4, 1, 1);
hold on;
plot(alpha, qe.mean(:, 1));
plot(alpha(im), qe.mean(im,1), '*r');
plot(alpha, ones(size(alpha))*true_mean(1), '--k');
title('Mean of quaternion error against alpha');
ylabel('$\bar q_1$', 'interpreter', 'latex');
grid;
subplot(4, 1, 2);
hold on;
plot(alpha, qe.mean(:, 2));
plot(alpha(im), qe.mean(im,2), '*r');
plot(alpha, ones(size(alpha))*true_mean(2), '--k');
ylabel('$\bar q_2$', 'interpreter', 'latex');
grid;
subplot(4, 1, 3);
hold on;
plot(alpha, qe.mean(:, 3));
plot(alpha(im), qe.mean(im,3), '*r');
plot(alpha, ones(size(alpha))*true_mean(3), '--k');
ylabel('$\bar q_3$', 'interpreter', 'latex');
grid;
subplot(4, 1, 4);
hold on;
plot(alpha, qe.mean(:, 4));
plot(alpha(im), qe.mean(im,4), '*r');
plot(alpha, ones(size(alpha))*true_mean(4), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\bar q_4$', 'interpreter', 'latex');
grid;

figure;
subplot(4, 1, 1);
hold on;
plot(alpha, qe.cov(:, 1, 1));
plot(alpha(ic), qe.cov(ic, 1, 1), '*r');
plot(alpha, ones(size(alpha))*true_cov(1,1), '--k');
title('Variance of quaternion error against alpha');
ylabel('$\mathrm{Var}(q_1)$', 'interpreter', 'latex');
grid;
subplot(4, 1, 2);
hold on;
plot(alpha, qe.cov(:, 2, 2));
plot(alpha(ic), qe.cov(ic, 2, 2), '*r');
plot(alpha, ones(size(alpha))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(q_2)$', 'interpreter', 'latex');
grid;
subplot(4, 1, 3);
hold on;
plot(alpha, qe.cov(:, 3, 3));
plot(alpha(ic), qe.cov(ic, 3, 3), '*r');
plot(alpha, ones(size(alpha))*true_cov(3,3), '--k');
ylabel('$\mathrm{Var}(q_3)$', 'interpreter', 'latex');
grid;
subplot(4, 1, 4);
hold on;
plot(alpha, qe.cov(:, 4, 4));
plot(alpha(ic), qe.cov(ic, 4, 4), '*r');
plot(alpha, ones(size(alpha))*true_cov(4,4), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\mathrm{Var}(q_4)$', 'interpreter', 'latex');
grid;


%% Euler angles error
close all

if ~exist('results') || ~exist('time')
    load('results.mat');
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

alpha = extractfield(results, 'alpha');
alpha = alpha(2:end);

[~, im] = min(vecnorm(eul.mean(:, 1:3), 2, 2));
[~, ic] = min(sum(sum(abs(eul.cov), 3), 2));


figure;
subplot(3, 1, 1);
hold on;
plot(alpha, eul.mean(:, 1));
plot(alpha(im), eul.mean(im,1), '*r');
plot(alpha, ones(size(alpha))*true_mean(1), '--k');
title('Mean of euler angle error against alpha');
ylabel('$\bar \delta_\phi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 2);
hold on;
plot(alpha, eul.mean(:, 2));
plot(alpha(im), eul.mean(im,2), '*r');
plot(alpha, ones(size(alpha))*true_mean(2), '--k');
ylabel('$\bar \delta_\theta [\mathrm{rad}]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 3);
hold on;
plot(alpha, eul.mean(:, 3));
plot(alpha(im), eul.mean(im,3), '*r');
plot(alpha, ones(size(alpha))*true_mean(3), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\bar \delta_\psi [\mathrm{rad}]$', 'interpreter', 'latex');
grid;

figure;
subplot(3, 1, 1);
hold on;
plot(alpha, eul.cov(:, 1, 1));
plot(alpha(ic), eul.cov(ic, 1, 1), '*r');
plot(alpha, ones(size(alpha))*true_cov(1,1), '--k');
title('Variance of euler angle error against alpha');
ylabel('$\mathrm{Var}(\delta_\phi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 2);
hold on;
plot(alpha, eul.cov(:, 2, 2));
plot(alpha(ic), eul.cov(ic, 2, 2), '*r');
plot(alpha, ones(size(alpha))*true_cov(2,2), '--k');
ylabel('$\mathrm{Var}(\delta_\theta) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;
subplot(3, 1, 3);
hold on;
plot(alpha, eul.cov(:, 3, 3));
plot(alpha(ic), eul.cov(ic, 3, 3), '*r');
plot(alpha, ones(size(alpha))*true_cov(3,3), '--k');
xlabel('$\alpha$', 'interpreter', 'latex');
ylabel('$\mathrm{Var}(\delta_\psi) [\mathrm{rad}^2]$', 'interpreter', 'latex');
grid;


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