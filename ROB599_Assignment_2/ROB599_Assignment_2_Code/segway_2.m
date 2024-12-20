clear
clc
close all

%% System Parameters

param.m = 0.2;
param.M = 0.5;
param.J = 0.006;
param.l = 0.3;
param.c = 0.1;
param.gamma = 0.1;
param.g = 9.81;
param.Jt= param.J + param.m*param.l^2;
param.Mt= param.M + param.m ;


%% Simulation Parameters
tSim = 0:0.1:10;
xo = [0; 0; pi/8; 0];

[tOut, xOut] = ode45(@(t,x) segway(t, x, param), tSim, xo);


% Calculate the applied force F(t) at each time step
F_t = zeros(length(tOut), 1);
for i = 1:length(tOut)
    F_t(i) = calculate_force(tOut(i));
end

% Create a new figure for the updated plots
figure('Position', [100, 100, 800, 900]); % Adjust figure size (width x height)

% Plot Cart Position (x1)
subplot(4,1,1);
plot(tOut, xOut(:,1), 'LineWidth', 2, 'Color', 'b');
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('x_1 [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('x_1 (Cart Pose) vs Time', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12); % Increase font size of x and y ticks
grid on;

% Plot Cart Velocity (x2)
subplot(4,1,2);
plot(tOut, xOut(:,2), 'LineWidth', 2, 'Color', 'g');
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('x_2 [m/s]', 'FontSize', 12, 'FontWeight', 'bold');
title('x_2 (Cart Velocity) vs Time', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12); % Increase font size of x and y ticks
grid on;

% Plot Pendulum Angle (x3)
subplot(4,1,3);
plot(tOut, xOut(:,3), 'LineWidth', 2, 'Color', [1 0.5 0]);
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('x_3 [rad]', 'FontSize', 12, 'FontWeight', 'bold');
title('x_3 (Pendulum Angle) vs Time', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12); % Increase font size of x and y ticks
grid on;

% Plot Pendulum Angular Velocity (x4)
subplot(4,1,4);
plot(tOut, xOut(:,4), 'LineWidth', 2, 'Color', 'm');
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('x_4 [rad/s]', 'FontSize', 12, 'FontWeight', 'bold');
title('x_4 (Pendulum Angular Velocity) vs Time', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12); % Increase font size of x and y ticks
grid on;

% Adjust figure to have a clean layout and appropriate subplot spacing
set(gcf, 'Position', [100, 100, 800, 900]); 
set(gcf, 'Color', 'w'); % Set white background for clarity



%% Calculate the Force Applied to the Cart
function F = calculate_force(t)
    if (t < 0)
        F = 0;
    elseif (t <= 2.5)
        F = 0.1 * cos(10 * t);
    elseif (t < 5)
        F = 0.5 * cos(5 * t);
    else
        F = 1.0 * cos(5 * t);
    end
end

%% State Space Of the System
function xdot = segway(t, x, param)

    % State variables
    x1 = x(1); % Cart position
    x2 = x(2); % Cart velocity
    x3 = x(3); % Pendulum angle
    x4 = x(4); % Pendulum angular velocity

    F = calculate_force(t);

    x1dot = x2;
    x2dot = (F*(param.J + param.m*param.l^2) - (param.J + param.m*param.l^2)*param.c*x2 - (param.J + param.m*param.l^2)*param.l*param.m*x4^2*sin(x3) + param.g*param.l^2*param.m^2*sin(2*x3)/2 - param.gamma*param.l*param.m*x4*cos(x3))/((param.J + param.m*param.l^2)*(param.M+param.m) - param.l^2*param.m^2*cos(x3)*cos(x3));
    x3dot = x4;
    x4dot = (F*param.l*param.m*cos(x3) + (param.M+param.m)*param.g*param.l*param.m*sin(x3) - (param.M+param.m)*param.gamma*x4 - param.c*param.l*param.m*x2*cos(x3) - param.l^2*param.m^2*x4^2*sin(2*x3)/2)/((param.J + param.m*param.l^2)*(param.M+param.m) - param.l^2*param.m^2*cos(x3)*cos(x3));

    xdot = [x1dot; x2dot; x3dot; x4dot];

end