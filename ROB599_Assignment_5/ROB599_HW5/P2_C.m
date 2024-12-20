clear all
close all
clc

I = 0.0693;  % Inertia
b = 0.417;   % Damping coefficient
G = tf(9, [I, b, 0]);  % Open-loop transfer function
C = tf([2.04, 18.714],[1, 18.714]);
L = series(C, G);

% Generate the Bode plot of plant with gain and phase margins
%margin(G);
%grid on;

% Generate the Bode plot of lead controller and plant with gain and phase margins
margin(L);
grid on;

