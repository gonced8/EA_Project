% Obtain data in double array for alpha vals and cell arrays for the rest from results.mat
get_data;


%% Plot

if ~exist('results') || ~exist('time')
    get_data;
end

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

if ~exist('results') || ~exist('time')
    get_data;
end


% Plot trace of Q over time for values of alpha
alpha_vec = [0, 0.25, 0.5, 0.75, 1];
n_alpha = length(alpha_vec);

index = zeros(1,n_alpha);

for i = 1:n_alpha
    [~,index(i)] = min(abs(alpha - alpha_vec(i)));
end

figure; hold on; grid on;
title('Trace of Q matrix for several values of \alpha');

for j = 1:n_alpha
    
    Q = kalman_Q{index(j)};
    ndata = length(time);
    trace_Q = zeros(ndata,1);
    eig_Q = zeros(ndata,6);
    
    for i = 1:ndata
        Q_i = reshape(Q(i,:,:), size(Q,2), size(Q,3));
        trace_Q(i) = trace(Q_i);
        eig_Q(i,:) = eig(Q_i);
    end
    
    subplot(n_alpha,1,j);
    plot(time, trace_Q);
    set(gca, 'YScale', 'log');
    xlabel('Time [$s$]','Interpreter','latex','fontsize',12.0)
    ylabel(['Trace of Q for \alpha = ' num2str(alpha_vec(j))]);
end


% Plot variance of Q for each alpha
for i = 1:npoints
    
    
    
end