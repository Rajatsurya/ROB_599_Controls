clear all
close all
clc

% Parameters
I = 0.0693;  % Inertia
b = 0.417;   % Damping coefficient
G = tf(9, [I, b, 0]);  % Open-loop transfer function
C = tf([2.04, 18.714], [1, 18.714]);  % Controller transfer function
L = series(C, G);  % Open-loop transfer function with controller

% Check if the Gain Margin and Phase Margin are calculated
[Gm, Pm, Wcg, Wcp] = margin(L);
disp(['Gain Margin (dB): ', num2str(20*log10(Gm))]);
disp(['Phase Margin (deg): ', num2str(Pm)]);
disp(['Gain Crossover Frequency (rad/s): ', num2str(Wcg)]);
disp(['Phase Crossover Frequency (rad/s): ', num2str(Wcp)]);

% Plot the Bode plot with margins
margin(L);
title('Bode Plot of the Lead Controller and Plant');
grid on;
