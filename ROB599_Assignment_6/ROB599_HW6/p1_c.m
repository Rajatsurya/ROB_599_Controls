clear all
close all
clc

% Given system parameters
Jm = 0.0097; % kg·m^2
Jl = 0.0097; % kg·m^2
bm = 0.04169; % Ns·m^−1
bl = 0.04169; % Ns·m^−1
k = 100; % Nm·rad^−1

% Define state-space matrices A and B
A = [0, 1, 0, 0;
    -k/Jl, -bl/Jl, k/Jl, 0;
     0, 0, 0, 1;
     k/Jm, 0, -k/Jm, -bm/Jm];
B = [0; 0; 0; 1/Jm];

% Define Q and R
Q = diag([5, 0.1, 5, 0.1]); % State weighting matrix
R = 0.1; % Control input weighting

% Compute the optimal gain matrix K using LQR
K = lqr(A, B, Q, R);

% Display the result
disp('Optimal feedback gain matrix K:');
disp(K);
