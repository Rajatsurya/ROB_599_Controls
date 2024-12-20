% To simulate the respones of the system for 4 seconds
%initial condition is 0 
%q1dot and q2dot at t=0 is 0
% for the desired value qd1 and qd2 The reference input qd1 and qd2 should be a step from 0 to π/2
radians
Kp1=50;
Kp2=50;
Kd1=10;
Kd2=10;
q1 = x(1); q1dot = x(2); q2 = x(3); q2dot = x(4);
% You can also define your physical parameters here so you don't need
% the input argument 'para'
m1 = para.m1; m2 = para.m2; I1 = para.I1; I2 = para.I2; g = para.g;
l1 = para.l1; lc1 = para.lc1; lc2 = para.lc2;
% Mass/inertia matrix
D = zeros(2, 2);
D(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2))+I1+I2;
D(1,2) = m2*(lc2^2+l1*lc2*cos(q2))+I2;
D(2,1) = D(1,2);
D(2,2) = m2*lc2^2+I2;
% Coriolis matrix
C = zeros(2, 2);
C(1, 1) = -m2*l1*lc2*sin(q2)*q2dot;
C(1, 2) = -m2*l1*lc2*sin(q2)*(q1dot + q2dot);
C(2, 1) = m2*l1*lc2*sin(q2)*q1dot;
C(2, 2) = 0;
% Gravity terms
N = zeros(2, 1);
N(1) = m1*g*lc1*cos(q1) + m2*g*(l1*cos(q1) + lc2*cos(q1 + q2));
N(2) = m2*g*lc2*cos(q1 + q2);
tau = zeros(2,1);
tau(1) = Kp1*(q1d-q1)-KD1*q1dot;
tau(2) = Kp2*(q2d-q2)-KD2*q2dot;
a1 = tau(1) - C(1, 1)*q1dot - C(1, 2)*q2dot - N(1);
a2 = tau(2) - C(2, 1)*q1dot - N(2);
xdot = [x(2);
1/det(D)*(D(2,2)*a1 - D(1,2)*a2);
x(4);
1/det(D)*(-D(2,1)*a1 + D(1,1)*a2);];
%ODE 45 to slove the differential requation

%Plot the following 
%joint responses, joint torques, and tracking errors for each joint

