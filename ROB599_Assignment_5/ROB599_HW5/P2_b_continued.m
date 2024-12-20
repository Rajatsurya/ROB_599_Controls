clear all
close all
clc

% Define the time vector for 0.3 seconds
t = 0:0.001:0.3; % Time vector from 0 to 0.3 seconds with a step size of 1 ms
% Define the steady-state value of the ideal response
steady_state_value = 1; % Replace with your system's steady-state value if different

% Define Kd and Kp based on root locus analysis
Kd = 0.701;  % value of Kd obtained from the root locus
Kp = 10 * Kd;
I = 0.0693;  % Inertia
b = 0.417;   % Damping coefficient
C = tf([Kd, Kd*10], 1);  % PD controller
G = tf(1, [I, b, 0]);  % Open-loop transfer function
L = series(C, G);  % Open-loop transfer function with controller

% Closed-loop transfer function
sys = feedback(L, 1);

% Step response
figure;
step(sys);
title('Step Response of the Closed-Loop System');
grid on;

