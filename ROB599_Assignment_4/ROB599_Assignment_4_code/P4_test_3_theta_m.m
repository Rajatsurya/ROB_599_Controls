% Clear previous data
clear all;
close all;
clc;

% Define the transfer function numerator and denominator
% KD will be used as the variable gain
numerator = [0.0097, 0.13869, 100.4169, 1000];  % Represents KD * (100s + 1000), with KD as the gain

% Denominator coefficients in terms of KD (KD is the gain variable)
denominator = [
    0.00009409, 0.000808786,1.94173,8.34775,0
];

% Create the transfer function with KD as the varying gain
sys = tf(numerator, denominator);

% Plot the root locus with KD as the varying gain
rlocus(sys);
title('Root Locus of H(s) with KD as the varying gain and theta_m as feedback');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
