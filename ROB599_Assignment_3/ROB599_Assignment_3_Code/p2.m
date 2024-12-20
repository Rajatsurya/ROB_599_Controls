%P2

% Define the time span and initial conditions
tspan = [0 4]; % from 0 to 4 seconds
x0 = [0; 0; 0; 0]; % initial conditions for [q1, q1dot, q2, q2dot]

% Define the desired trajectory as a function of time
function qd = desired_trajectory(t)
    if t < 2
        qd = [pi/2; pi/2]; % step to π/2 radians
    else
        qd = [0; 0]; % step back to 0 radians
    end
end

% Define the system of differential equations
function xdot = robot_dynamics(t, x, para)
    % Extract joint positions and velocities
    q1 = x(1); q1dot = x(2); q2 = x(3); q2dot = x(4);
    
    % Desired trajectory at time t
    qd = desired_trajectory(t);
    q1d = qd(1);
    q2d = qd(2);

    % Mass/inertia matrix D(q)
    D = zeros(2, 2);
    D(1,1) = para.m1*para.lc1^2 + para.m2*(para.l1^2 + para.lc2^2 + 2*para.l1*para.lc2*cos(q2)) + para.I1 + para.I2;
    D(1,2) = para.m2*(para.lc2^2 + para.l1*para.lc2*cos(q2)) + para.I2;
    D(2,1) = D(1,2);
    D(2,2) = para.m2*para.lc2^2 + para.I2;

    % Coriolis matrix C(q,qdot)
    C = zeros(2, 2);
    C(1, 1) = -para.m2*para.l1*para.lc2*sin(q2)*q2dot;
    C(1, 2) = -para.m2*para.l1*para.lc2*sin(q2)*(q1dot + q2dot);
    C(2, 1) = para.m2*para.l1*para.lc2*sin(q2)*q1dot;
    C(2, 2) = 0;

    % Gravity terms N(q)
    N = zeros(2, 1);
    N(1) = para.m1*para.g*para.lc1*cos(q1) + para.m2*para.g*(para.l1*cos(q1) + para.lc2*cos(q1 + q2));
    N(2) = para.m2*para.g*para.lc2*cos(q1 + q2);

    % PD control torques
    tau = zeros(2, 1);
    tau(1) = para.Kp1*(q1d - q1) - para.Kd1*q1dot;
    tau(2) = para.Kp2*(q2d - q2) - para.Kd2*q2dot;

    % Limit the torques to the range [-50, 50]
    tau(1) = max(min(tau(1), 50), -50);
    tau(2) = max(min(tau(2), 50), -50);

    % Calculate accelerations
    a = tau - C * [q1dot; q2dot] - N;
    
    % Solve for joint accelerations using inverse of D matrix
    qddot = D \ a;

    % Return derivative of state vector
    xdot = [q1dot; qddot(1); q2dot; qddot(2)];
end

% Define parameters for the robot arm
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

% Solve the differential equations using ODE45
[t, x] = ode45(@(t,x) robot_dynamics(t, x, para), tspan, x0);

% Calculate torques and tracking errors over time
tau1 = zeros(length(t), 1);
tau2 = zeros(length(t), 1);
tracking_error_q1 = zeros(length(t), 1);
tracking_error_q2 = zeros(length(t), 1);

for i = 1:length(t)
    qd = desired_trajectory(t(i));
    q1d = qd(1);
    q2d = qd(2);
    
    % Extract current states
    q1 = x(i, 1);
    q1dot = x(i, 2);
    q2 = x(i, 3);
    q2dot = x(i, 4);
    
    % Calculate PD control torques
    tau1(i) = para.Kp1 * (q1d - q1) - para.Kd1 * q1dot;
    tau2(i) = para.Kp2 * (q2d - q2) - para.Kd2 * q2dot;
    
    % Limit the torques to the range [-50, 50]
    tau1(i) = max(min(tau1(i), 50), -50);
    tau2(i) = max(min(tau2(i), 50), -50);

    % Calculate tracking errors
    tracking_error_q1(i) = q1d - q1;
    tracking_error_q2(i) = q2d - q2;
end

% Plot joint responses with desired trajectory
figure;
subplot(3, 1, 1);
plot(t, x(:, 1), 'r', t, x(:, 3), 'b');
hold on;
qd_plot = arrayfun(@desired_trajectory, t, 'UniformOutput', false);
qd1 = cellfun(@(qd) qd(1), qd_plot); % Extract q1d for all time steps
qd2 = cellfun(@(qd) qd(2), qd_plot); % Extract q2d for all time steps
plot(t, qd1, '--r', t, qd2, '--b'); % Plot desired trajectories
hold off;
title('Joint Angles and Desired Trajectories');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('q_1', 'q_2', 'q_{1d}', 'q_{2d}');

% Plot joint torques
subplot(3, 1, 2);
plot(t, tau1, 'r', t, tau2, 'b');
title('Joint Torques');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('\tau_1', '\tau_2');
ylim([-50 50]);  % Set y-axis limits from -100 to +100


% Plot tracking errors
subplot(3, 1, 3);
plot(t, tracking_error_q1, 'r', t, tracking_error_q2, 'b');
hold on;
plot(t, zeros(size(t)), '--k'); % Plot zero line for perfect tracking
hold off;
title('Tracking Errors');
xlabel('Time (s)');
ylabel('Error (rad)');
legend('Error q_1', 'Error q_2');

disp("In theory, the answer is yes, but with certain conditions and limitations:")

disp("Proportional Gain (Kp):")
disp("Increasing Kp can reduce the steady-state error. A higher Kp will make the system more responsive by increasing the corrective torque based on the error in position.")
disp("However, too high of a Kp can cause overshoot and oscillations, where the system overshoots the desired value and keeps oscillating around it before settling down.")

disp("Derivative Gain (Kd):")
disp("Increasing Kd improves the damping of the system. It reduces oscillations and overshoot by responding to the rate of change of the error (i.e., velocity).")
disp("However, if Kd is too high, it can make the system sluggish or even unstable in the presence of noise, since the derivative term amplifies high-frequency signals (like noise or rapid changes).")

disp("Eliminating the Error:")
disp("For a PD controller, there is no integral term, so steady-state error might not completely disappear unless the proportional and derivative gains are perfectly tuned.")
disp("A PI (Proportional-Integral) or PID controller (adding an integral term) is typically required to eliminate steady-state errors completely.")
disp("With PD control, reducing the tracking error largely depends on how well-tuned Kp and Kd are. The best you can achieve is minimizing the error, but it may not be possible to eliminate it entirely without integral action.")
