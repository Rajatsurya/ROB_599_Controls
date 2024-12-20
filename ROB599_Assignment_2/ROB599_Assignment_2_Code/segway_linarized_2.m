%% ROB599-HW1-Problem7-A
% Simulating pendulum on a cart using a linearised model (linearized only @
% equillibrium x=0, u=0.
%% Cleanup
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

%% Simulation Parameters
to = 0;
tFinal = 10;
tSim = 0:0.01:10;
xo = [0; 0; pi/8; 0];

%% Simulation
[tOut, xOut] = ode45(@(t,x) segwayLinearized(t, x, param), tSim, xo);


% Calculate the applied force F(t) at each time step
F_t = zeros(length(tOut), 1);
for i = 1:length(tOut)
    F_t(i) = calculate_force(tOut(i));
end

%% Show the Plots with Improved Layout and Save to PNG
figure('Position', [100, 100, 1200, 900]);  % Set figure size (Width: 1200, Height: 900)

% Add a super-title for the whole figure
sgtitle('Segway System Dynamics Simulation. Variable External Force.', 'FontSize', 24, 'FontWeight', 'bold');

% Plot Cart Position
subplot(5,1,1);  % Create the first subplot in a 5-row grid
plot(tOut, xOut(:,1), 'LineWidth', 3, 'Color', 'b');
ylabel('Cart Pose (m)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 12);
grid on;

% Plot Cart Velocity
subplot(5,1,2);  % Create the second subplot
plot(tOut, xOut(:,2), 'LineWidth', 3, 'Color', 'g');
ylabel('Cart Vel (m/s)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 12); 
grid on;

% Plot Pendulum Angle
subplot(5,1,3);  % Create the third subplot
plot(tOut, xOut(:,3), 'LineWidth', 3, 'Color', [1 0.5 0]);
ylabel('Pend. Angle (rad)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 12);
grid on;

% Plot Pendulum Angular Velocity
subplot(5,1,4);  % Create the fourth subplot
plot(tOut, xOut(:,4), 'LineWidth', 3, 'Color', 'm');
ylabel('Pendulum Ang. Vel. (rad/s)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 12);
grid on;

% Plot Applied Force
subplot(5,1,5);  % Create the fifth subplot
plot(tOut, F_t, 'LineWidth', 3, 'Color', 'r');
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Force (N)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 12);
grid on;

% Save the figure as a high-resolution PNG image
print(gcf, 'SegwayPlot.png', '-dpng', '-r300');



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

%% State Space Of the System (Linearized)
function xdot = segwayLinearized(t, x, param)

    % State variables
    x1 = x(1);
    x2 = x(2);
    x3 = x(3); 
    x4 = x(4);

    F = calculate_force(t);
    
    
    x1dot = x2;
    x2dot = (1/((param.J + param.m*param.l^2)*(param.M+param.m) - param.l^2*param.m^2 )) * ( -((param.J + param.m*param.l^2)*param.c) * x2   +        (param.g*param.l^2*param.m^2)       * x3   -  (param.gamma*param.l*param.m)  * x4        + (param.J + param.m*param.l^2) * F) ;
    x3dot = x4;
    x4dot = (1/((param.J + param.m*param.l^2)*(param.M+param.m) - param.l^2*param.m^2 )) * (       -(param.c*param.l*param.m)         * x2   +((param.M+param.m)*param.g*param.l*param.m) * x3   - ((param.M+param.m)*param.gamma) * x4        + (param.m*param.l)             * F);

    xdot = [x1dot; x2dot; x3dot; x4dot];

end