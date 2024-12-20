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
Q = diag([1, 0.1, 1, 0.1]); % State weighting matrix
R = 1; % Control input weighting

% Compute the optimal gain matrix K using LQR
K = lqr(A, B, Q, R);

% Initial state vector
x0 = [pi/2; 0; pi/2; 0]; % Corrected to 4 states

% Simulate the system using ODE45
[T, X] = ode45(@(t, x) myode(t, x, A, B, K), 0:0.01:5, x0);

% Plot the state variables
figure;
plot(T, X(:, 1), 'b', T, X(:, 3), 'r--', 'LineWidth', 1.5);
xlabel('Time [sec]');
ylabel('Radians', 'Interpreter', 'latex');
legend({'$\theta_l(t)$- Load Angle', '$\theta_m(t)$- Motor Angle'}, 'Interpreter', 'latex');
grid on;
title('Load Angle and Motor Angle Position')

figure;
plot(T, X(:, 2), 'b', T, X(:, 4), 'r-', 'LineWidth', 1.5);
xlabel('Time [sec]');
ylabel('Rad/sec', 'Interpreter', 'latex');
legend({'$\dot{\theta_l}(t)$ - Load Velocity', '$\dot{\theta_m}(t)$ - Motor Velocity'}, 'Interpreter', 'latex');
grid on;
title('Load and Motor Velocities')

% Compute and plot the control input
U = zeros(length(T), 1); % Preallocate control input array
for k = 1:length(T)
    U(k) = -K * X(k, :)';
end

figure;
plot(T, U, 'k', 'LineWidth', 1.5);
xlabel('Time [sec]');
ylabel('Control Input u(t)');
legend('u(t)');
grid on;

% ODE function
function [xdot] = myode(t, x, A, B, K)
    u = -K * x; % Control input
    xdot = A * x + B * u; % State-space equations
end
