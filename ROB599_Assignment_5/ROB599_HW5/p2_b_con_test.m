clear all
close all
clc

% Define the time vector for 0.3 seconds
t = 0:0.001:0.8; % Time vector from 0 to 0.3 seconds with a step size of 1 ms

% Define the steady-state value of the ideal response
steady_state_value = 1; % Replace with your system's steady-state value if different
ideal_response = steady_state_value * ones(size(t)); % Ideal step response (constant)

% Define system parameters
Kd = 0.701;  % Value of Kd obtained from the root locus
Kp = 10 * Kd; % Proportional gain based on Kd
I = 0.0693;  % Inertia
b = 0.417;   % Damping coefficient

% Define transfer functions
C = tf([Kd, Kp], 1); % PD controller
G = tf(1, [I, b, 0]); % Open-loop transfer function (plant)
L = series(C, G); % Open-loop transfer function with controller

% Closed-loop transfer function
sys = feedback(L, 1); % Feedback system with unity feedback

% Overlay both step responses for comparison
figure;
hold on;
step(sys, t); % Closed-loop system response
plot(t, ideal_response, '--r', 'LineWidth', 1.5); % Ideal step response
hold off;
xlabel('Time (s)');
ylabel('Amplitude');
title('Comparison of Step Responses');
legend('Closed-Loop Step Response', 'Ideal Step Response');
grid on;
