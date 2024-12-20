function [qd1, vd1, ad1, qd2, vd2, ad2] = generate_trajectory(t)
    % Time breakpoints
    t_waypoints = [0 2 4];

    % Waypoints for position q(t) for both joints
    waypoints = [0, pi/2, 0];   % Desired positions for both joints

    % Desired velocities for both joints
    velocities = [0, 0, 0];     % Desired velocities for both joints

    % Generate cubic polynomial trajectory
    [qd, vd, ad] = cubicpolytraj(waypoints, t_waypoints, t, ...
                                 'VelocityBoundaryCondition', velocities);

    % Assign the same trajectory to both joints
    qd1 = qd; vd1 = vd; ad1 = ad;  % For joint 1
    qd2 = qd; vd2 = vd; ad2 = ad;  % For joint 2
end

function xdot = robot_dynamics(t, x, para)
    % Extract joint positions and velocities
    q1 = x(1); q1dot = x(2); q2 = x(3); q2dot = x(4);
    
    % Desired trajectory at time t
    [qd, vd, ad] = generate_trajectory(t);  % Get desired position, velocity, acceleration
    q1d = qd;  % Same trajectory for both q1 and q2
    q2d = qd;

    % Ensure that vd and ad are vectors with two identical elements
    Vd = [vd; vd];  % Both joints have the same desired velocity
    Ad = [ad; ad];  % Both joints have the same desired acceleration

    Aq = zeros(2,1);
    Aq(1)= Ad(1) + para.Kp1*(q1d - q1) + para.Kd1*(Vd(1)-q1dot);
    Aq(2)= Ad(2) + para.Kp2*(q2d - q2) + para.Kd2*(Vd(2)-q2dot);

    
    % Mass/inertia matrix D(q)
    D = zeros(2, 2);
    D(1,1) = para.m1 * para.lc1^2 + para.m2 * (para.l1^2 + para.lc2^2 + 2 * para.l1 * para.lc2 * cos(q2)) + para.I1 + para.I2;
    D(1,2) = para.m2 * (para.lc2^2 + para.l1 * para.lc2 * cos(q2)) + para.I2;
    D(2,1) = D(1,2);
    D(2,2) = para.m2 * para.lc2^2 + para.I2;

    % Coriolis matrix C(q,qdot)
    C = zeros(2, 2);
    C(1, 1) = -para.m2 * para.l1 * para.lc2 * sin(q2) * q2dot;
    C(1, 2) = -para.m2 * para.l1 * para.lc2 * sin(q2) * (q1dot + q2dot);
    C(2, 1) = para.m2 * para.l1 * para.lc2 * sin(q2) * q1dot;
    C(2, 2) = 0;

    % Gravity terms N(q)
    N = zeros(2, 1);
    N(1) = para.m1 * para.g * para.lc1 * cos(q1) + para.m2 * para.g * (para.l1 * cos(q1) + para.lc2 * cos(q1 + q2));
    N(2) = para.m2 * para.g * para.lc2 * cos(q1 + q2);
    
    % Feed back new controller
    Tau = D*Aq + C*[q1dot; q2dot] + N;

    % Limit the torques to the range [-50, 50]
    Tau(1) = max(min(Tau(1), 50), -50);
    Tau(2) = max(min(Tau(2), 50), -50);

    % Calculate accelerations
    a = [Tau(1); Tau(2)] - C * [q1dot; q2dot] - N;
    
    % Solve for joint accelerations using inverse of D matrix
    qdoubledot = D \ a;

    % Return derivative of state vector
    xdot = [q1dot; qdoubledot(1); q2dot; qdoubledot(2)];
end

% Define the parameters in the 'para' structure
para.m1 = 7.848;
para.m2 = 4.49;
para.l1 = 0.3;
para.lc1 = 0.1554;
para.lc2 = 0.0341;
para.I1 = 0.176;
para.I2 = 0.0411;
para.g = 9.81;
para.Kp1 = 50;
para.Kp2 = 50;
para.Kd1 = 10;
para.Kd2 = 10;

% Define the simulation time span and initial conditions
tspan = [0 4];  % Define the time span for simulation
x0 = [0; 0; 0; 0];  % Initial conditions for [q1; q1dot; q2; q2dot]

% Run the simulation using ODE45
[t, x] = ode45(@(t, x) robot_dynamics(t, x, para), tspan, x0);

% Initialize arrays for accelerations
joint_accelerations = zeros(length(t), 2); % For both joints

% Calculate torques and tracking errors over time
tau1 = zeros(length(t), 1);
tau2 = zeros(length(t), 1);
tracking_error_q1 = zeros(length(t), 1);
tracking_error_q2 = zeros(length(t), 1);

for i = 1:length(t)
    % Desired trajectory at time t(i)
    [qd, vd, ad] = generate_trajectory(t(i));  % Get desired trajectory
    q1d = qd;  % Desired position for joint 1
    q2d = qd;  % Desired position for joint 2
    
    % Ensure that vd and ad are scalars
    Vd = [vd; vd];  % Same velocity for both joints
    Ad = [ad; ad];  % Same acceleration for both joints
    
    % Extract current states
    q1 = x(i, 1);
    q1dot = x(i, 2);
    q2 = x(i, 3);
    q2dot = x(i, 4);

    % Calculate Aq using the same method as in robot_dynamics
    Aq = zeros(2,1);
    Aq(1) = Ad(1) + para.Kp1 * (q1d - q1) + para.Kd1 * (Vd(1) - q1dot);
    Aq(2) = Ad(2) + para.Kp2 * (q2d - q2) + para.Kd2 * (Vd(2) - q2dot);

    % Calculate the mass matrix D for the current state
    D_current = zeros(2, 2);
    D_current(1,1) = para.m1 * para.lc1^2 + para.m2 * (para.l1^2 + para.lc2^2 + 2 * para.l1 * para.lc2 * cos(q2)) + para.I1 + para.I2;
    D_current(1,2) = para.m2 * (para.lc2^2 + para.l1 * para.lc2 * cos(q2)) + para.I2;
    D_current(2,1) = D_current(1,2);
    D_current(2,2) = para.m2 * para.lc2^2 + para.I2;

    % Coriolis matrix C for the current state
    C_current = zeros(2, 2);
    C_current(1, 1) = -para.m2 * para.l1 * para.lc2 * sin(q2) * q2dot;
    C_current(1, 2) = -para.m2 * para.l1 * para.lc2 * sin(q2) * (q1dot + q2dot);
    C_current(2, 1) = para.m2 * para.l1 * para.lc2 * sin(q2) * q1dot;
    C_current(2, 2) = 0;

    % Gravity terms N for the current state
    N_current = zeros(2, 1);
    N_current(1) = para.m1 * para.g * para.lc1 * cos(q1) + para.m2 * para.g * (para.l1 * cos(q1) + para.lc2 * cos(q1 + q2));
    N_current(2) = para.m2 * para.g * para.lc2 * cos(q1 + q2);

    % Calculate feed-forward torque
    Tau = D_current * Aq + C_current * [q1dot; q2dot] + N_current;

    % Compute control torques
    tau1(i) = Tau(1);
    tau2(i) = Tau(2);

    % Limit the torques to the range [-50, 50]
    tau1(i) = max(min(tau1(i), 50), -50);
    tau2(i) = max(min(tau2(i), 50), -50);

    % Compute joint accelerations using the current torque values
    a_current = [tau1(i); tau2(i)] - C_current * [q1dot; q2dot] - N_current;
    joint_accelerations(i, :) = D_current \ a_current;  % Joint accelerations

    % Track position and velocity errors
    tracking_error_q1(i) = q1d - q1;
    tracking_error_q2(i) = q2d - q2;
    tracking_error_v1(i) = Vd(1) - q1dot;
    tracking_error_v2(i) = Vd(2) - q2dot;

    % Track acceleration errors
    tracking_error_a1(i) = Ad(1) - joint_accelerations(i, 1);  % Acceleration error for joint 1
    tracking_error_a2(i) = Ad(2) - joint_accelerations(i, 2);  % Acceleration error for joint 2
end

% Plot joint positions
figure;

subplot(3, 1, 1);
plot(t, x(:, 1), 'b', 'LineWidth', 1.5);
hold on;
plot(t, x(:, 3), 'r', 'LineWidth', 1.5);
plot(t, qd, 'k--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Position (rad)');
title('Actual Joint Positions');
legend('Joint 1', 'Joint 2');
grid on;

% Plot joint velocities
subplot(3, 1, 2);
plot(t, x(:, 2), 'b', 'LineWidth', 1.5);
hold on;
plot(t, x(:, 4), 'r', 'LineWidth', 1.5);
plot(t, vd, 'k--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title('Actual Joint Velocities');
legend('Joint 1', 'Joint 2');
grid on;

% Plot joint accelerations
figure;
subplot(2, 1, 1);
plot(t, joint_accelerations(:, 1), 'b', 'LineWidth', 1.5); % Joint 1 acceleration
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
title('Joint 1 Acceleration');
grid on;

subplot(2, 1, 2);
plot(t, joint_accelerations(:, 2), 'r', 'LineWidth', 1.5); % Joint 2 acceleration
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
title('Joint 2 Acceleration');
grid on;

% Create figure for joint torques
figure;

% Plot the data
plot(t, tau1, 'b', 'LineWidth', 2);  % Joint 1 torque (blue line)
hold on;
plot(t, tau2, 'r', 'LineWidth', 2);  % Joint 2 torque (red line)

% Customize the appearance
xlabel('Time (sec)');  % Label for the x-axis
ylabel('Joint torque (Nm)');  % Label for the y-axis
title('Joint Torques');  % Title of the plot
legend('\tau_1', '\tau_2');  % Legend for the plot

% Adjust axis limits to match the second figure
ylim([-5, 30]);  % Y-axis limits for torque
xlim([0 4]);     % X-axis limits for time


% Plot tracking errors
figure;

subplot(2, 1, 1);
plot(t, tracking_error_q1, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Error (rad)');
title('Tracking Error for Joint 1');
grid on;

subplot(2, 1, 2);
plot(t, tracking_error_q2, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Error (rad)');
title('Tracking Error for Joint 2');
grid on;


% Define the time span for the trajectory
tspan = linspace(0, 4, 100);  % 100 points from 0 to 4 seconds

% Preallocate arrays for joint trajectories
qd1 = zeros(size(tspan));
vd1 = zeros(size(tspan));
ad1 = zeros(size(tspan));
qd2 = zeros(size(tspan));
vd2 = zeros(size(tspan));
ad2 = zeros(size(tspan));

% Generate the trajectory for each time point
for i = 1:length(tspan)
    [qd1(i), vd1(i), ad1(i), qd2(i), vd2(i), ad2(i)] = generate_trajectory(tspan(i));
end

% Plot desired positions for both joints
figure;
subplot(3, 1, 1);  % 3 rows, 1 column, 1st subplot
plot(tspan, qd1, 'LineWidth', 2);
hold on;
plot(tspan, qd2, 'LineWidth', 2);
title('Desired Position (q) for Both Joints');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Joint 1', 'Joint 2');
grid on;

% Plot desired velocities for both joints
subplot(3, 1, 2);  % 3 rows, 1 column, 2nd subplot
plot(tspan, vd1, 'LineWidth', 2);
hold on;
plot(tspan, vd2, 'LineWidth', 2);
title('Desired Velocity (v) for Both Joints');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('Joint 1', 'Joint 2');
grid on;

% Plot desired accelerations for both joints
subplot(3, 1, 3);  % 3 rows, 1 column, 3rd subplot
plot(tspan, ad1, 'LineWidth', 2);
hold on;
plot(tspan, ad2, 'LineWidth', 2);
title('Desired Acceleration (a) for Both Joints');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
legend('Joint 1', 'Joint 2');
grid on;

