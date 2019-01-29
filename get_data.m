load('results.mat');

npoints = length(results);
alpha = zeros(npoints,1);
kalman_quaternion = cell(npoints,1);
kalman_omega = cell(npoints,1);
kalman_bias = cell(npoints,1);
kalman_sigma = cell(npoints,1);
kalman_Q = cell(npoints,1);
MEKF_q_e = cell(npoints,1);
MEKF_euler_e = cell(npoints,1);
MEKF_euler = cell(npoints,1);

for i = 1:npoints
    alpha(i) = results(i).alpha;
    kalman_quaternion{i} = results(i).kalman_quaternion;
    kalman_omega{i} = results(i).kalman_omega;
    kalman_bias{i} = results(i).kalman_bias;
    kalman_sigma{i} = results(i).kalman_sigma;
    kalman_Q{i} = results(i).kalman_Q;
    MEKF_q_e{i} = results(i).MEKF_q_e;
    MEKF_euler_e{i} = results(i).MEKF_euler_e;
    MEKF_euler{i} = results(i).MEKF_euler;
end