% Define parameters
I = 0.069367;  % Inertia
b = 0.4166;   % Damping coefficient

% Define transfer function G(s) = 1 / (Is + b)
G = tf([1], [I, b]);

% Plot the Bode plot
margin(G);

% Add grid and title
grid on;
