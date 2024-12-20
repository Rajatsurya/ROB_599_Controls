clc;

% mass terms
syms m1 m2

% geometry terms
syms l1 l2 lc1 lc2

% Inertia Terms
syms I1 I2

% displacement
syms q1 q2

% velocity
syms q1dot q2dot

% acceleration
syms a1 a2

% Forces
syms tau1 tau2

% Gravitational constant
syms g

% Inertia matrix D
D = sym(zeros(2, 2));
D(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I1 + I2;
D(1,2) = m2*(lc2^2 + l1*lc2*cos(q2)) + I2;
D(2,1) = D(1,2);
D(2,2) = m2*lc2^2 + I2;

% Coriolis matrix C
C = sym(zeros(2, 2));
C(1, 1) = -m2*l1*lc2*sin(q2)*q2dot;
C(1, 2) = -m2*l1*lc2*sin(q2)*(q1dot + q2dot);
C(2, 1) = m2*l1*lc2*sin(q2)*q1dot;
C(2, 2) = 0;

% Gravity vector N
N = sym(zeros(2, 1));
N(1) = m1*g*lc1*cos(q1) + m2*g*(l1*cos(q1) + lc2*cos(q1 + q2));
N(2) = m2*g*lc2*cos(q1 + q2);

% Force vector T
T = sym(zeros(2, 1));
T(1) = tau1;
T(2) = tau2;

% Velocity vector V
V = sym(zeros(2, 1));
V(1) = q1dot;
V(2) = q2dot;

% Determinant of D
det_D = det(D);

% Compute accelerations a1 and a2 using determinant
a = (1/det_D) * [D(2,2), -D(1,2); -D(2,1), D(1,1)] * (T - C*V - N);
a1 = a(1);
a2 = a(2);

disp('Acceleration a1:');
disp(a1);
disp('Acceleration a2:');
disp(a2);

% Substituting given values for m1, m2, l1, lc1, lc2, I1, I2, g
m1_val = 7.848;
m2_val = 4.49;
l1_val = 0.3;
lc1_val = 0.1554;
lc2_val = 0.0341;
I1_val = 0.176;
I2_val = 0.0411;
g_val = 9.81;

% Substituting these values into the symbolic expressions
a1_val = subs(a1, [m1, m2, l1, lc1, lc2, I1, I2, g], [m1_val, m2_val, l1_val, lc1_val, lc2_val, I1_val, I2_val, g_val]);
a2_val = subs(a2, [m1, m2, l1, lc1, lc2, I1, I2, g], [m1_val, m2_val, l1_val, lc1_val, lc2_val, I1_val, I2_val, g_val]);

disp('Evaluated acceleration a1:');
disp(a1_val);
disp('Evaluated acceleration a2:');
disp(a2_val);
