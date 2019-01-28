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
