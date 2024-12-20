clear all
close all
clc

I = 0.0693;  % Inertia
b = 0.417;   % Damping coefficient
G = tf(1, [I, b, 0]);  % Open-loop transfer function

Kd = 1;  % Initial guess for Kd (we will adjust later)
Kp = 10 * Kd;  % Relationship Kp = 10 * Kd
C = tf([Kd, Kd*10], 1);  % PD controller
L = series(C, G);  % Open-loop transfer function with controller

figure;
rlocus(L);  % Plot the root locus
sgrid(0.8, 10);  % Overlay grid lines for ζ = 0.8 and ωn = 10 rad/s
title('Root Locus with Desired Poles');
grid on;


