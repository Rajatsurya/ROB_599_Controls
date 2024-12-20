% Clear previous data
clear all;
close all;
clc;

% Define the transfer function numerator and denominator
% KD will be used as the variable gain
numerator = [100, 1000];  % Represents KD * (100s + 1000), with KD as the gain

% Denominator coefficients in terms of KD (KD is the gain variable)
denominator = [
    0.00009409, ...
    (0.000808786 + 0.0097), ...
    (1.94173806 + 0.0097 * 10 + 0.04169), ...
    (8.338 + 0.04169 * 10 + 100), ...
    1000
];

% Create the transfer function with KD as the varying gain
sys = tf(numerator, denominator);

% Plot the root locus with KD as the varying gain
rlocus(sys);
title('Root Locus of H(s) with KD as the varying gain and theta_m as feedback');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
