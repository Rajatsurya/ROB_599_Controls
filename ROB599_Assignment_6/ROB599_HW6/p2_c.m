clear all
close all
clc

% System parameters
m = 1;    % kg
b = 2;    % Ns/m
k = 1;    % N/m

% State-space matrices
A = [0      1;
     -k/m  -b/m];
B = [0;
     1/m];
C = [1 0];  % Measurement matrix
K = [5 1];  % Feedback gain

% Process and measurement noise covariance
q1 = 10^-4;  % Process noise for position
q2 = 10^-4;  % Process noise for velocity
Qe = diag([q1, q2]);  % Process noise covariance matrix
Re = 0.1;             % Measurement noise covariance scalar
G = eye(2);           % Identity matrix for process noise

% Compute observer gain using LQE
L = lqe(A, G, C, Qe, Re);

% Initial conditions
x0 = [1; 0];         % True state
xhat0 = [1.2; 0.2];  % Estimated state

% Combine initial conditions
z0 = [x0; xhat0];    % Combined state 2x2 martix

% Time vector
tspan = 0:0.01:10;

% Simulate the system using ODE45
[T, Z] = ode45(@(t, z) combined_system(t, z, A, B, C, K, L), tspan, z0);

% Extract true state and estimated state
X = Z(:, 1:2);      % True state
X_hat = Z(:, 3:4);  % Estimated state

% Control input based on estimated state
U = zeros(length(T), 1);
for i = 1:length(T)
    U(i) = -K * X_hat(i, :)';
end

% Plot the true state and estimated state
% Plot the true state and estimated state for position
figure;
plot(T, X(:,1), 'b-', T, X_hat(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time [sec]', 'Interpreter', 'latex');
ylabel('Position $x(t)$', 'Interpreter', 'latex');
legend('True state $x(t)$', 'Estimated state $\hat{x}(t)$', 'Interpreter', 'latex');
grid on;

% Plot the true state and estimated state for velocity
figure;
plot(T, X(:,2), 'b-', T, X_hat(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time [sec]', 'Interpreter', 'latex');
ylabel('Velocity $\dot{x}(t)$', 'Interpreter', 'latex');
legend('True state $\dot{x}(t)$', 'Estimated state $\dot{\hat{x}}(t)$', 'Interpreter', 'latex');
grid on;

% Plot the control input
figure;
plot(T, U, 'k-', 'LineWidth', 1.5);
xlabel('Time [sec]', 'Interpreter', 'latex');
ylabel('Control Input $u(t)$', 'Interpreter', 'latex');
legend('$u(t)$', 'Interpreter', 'latex');
grid on;

% Compute and plot the state estimation error
error = X - X_hat;

figure;
plot(T, error(:,1), 'b-', T, error(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time [sec]', 'Interpreter', 'latex');
ylabel('Estimation Error', 'Interpreter', 'latex');
legend('$x(t) - \hat{x}(t)$', '$\dot{x}(t) - \dot{\hat{x}}(t)$', 'Interpreter', 'latex');
grid on;

eig(A-L*C)


% Combined system dynamics
function dzdt = combined_system(t, z, A, B, C, K, L)
    % Split z into true state (x) and estimated state (xhat)
    x = z(1:2);
    xhat = z(3:4);
    
    % Output measurement
    y = C * x;
    yhat = C*xhat; %measured value
    
    % Control input using the estimated state
    u = -K * xhat;
    
    % True system dynamics
    dxdt = A * x + B * u;
    
    % Observer dynamics
    dxhatdt = A*xhat + B*u + L*(y-yhat);
    
    % Combine derivatives
    dzdt = [dxdt; dxhatdt]; %2x2 matrix
end
