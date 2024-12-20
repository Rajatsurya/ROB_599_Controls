clear all
close all
clc

% System parameters from the problem
m = 1;    % kg
b = 2;    % Ns/m
k = 1;    % N/m

% Define state-space matrices A and B
A = [0      1;
     -k/m  -b/m];
B = [0;
     1/m];

% Feedback gain matrix K as given in the problem
K = [5 1];

% Initial conditions
x0 = [1; 0];

% Time vector
tspan = 0:0.01:10;

% Simulate the system using ODE45
[T, X] = ode45(@(t, x) myode(t, x, A, B, K), tspan, x0);

% Plot the state variables
figure;
plot(T, X(:,1), 'b-', T, X(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time [sec]');
ylabel('State Variables');
legend('x(t)', '$\dot{x}(t)$', 'Interpreter', 'latex');
grid on;

% Compute and plot the control input
U = zeros(length(T), 1);
for i = 1:length(T)
    U(i) = -K * X(i,:)';
end

figure;
plot(T, U, 'k-', 'LineWidth', 1.5);
xlabel('Time [sec]');
ylabel('Control Input');
legend('u(t)');
grid on;

% ODE function
function dxdt = myode(t, x, A, B, K)
    u = -K * x;
    dxdt = A * x + B * u;
end