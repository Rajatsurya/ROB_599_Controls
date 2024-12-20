% Clear previous data
clear all;
close all;
clc;

% Define the gain relationship
KD = 1; % Set KD to 1 initially; this will be varied by root locus
Kp = 10 * KD; % Fixed relationship between Kp and KD

% Define the coefficients of the transfer function based on H(s) provided
numerator = [100 * KD, 100 * Kp];

% Substitute Kp = 10 * KD in the denominator expression
denominator = [
    0.00009409, ...
    (0.000808786), ...
    (1.94173806), ...
    (8.338 + 100 * KD), ...
    100 * Kp
];

% Create a transfer function object with KD as a variable gain
sys = tf(numerator, denominator);

% Plot the root locus with KD as the varying gain
rlocus(sys);
title('Root Locus of H(s) with KD as the varying gain with theta_l as the feedback');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;