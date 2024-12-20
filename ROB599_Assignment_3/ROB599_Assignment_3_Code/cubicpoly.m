%% Cubic polynomial trajectory
% Author: Jiefu Zhang, courtesy of ROB510 material by Dr. Robert Gregg
% Date: Jan 24 24
%
% Input:
% t_pt: time at which a waypoint is passed (vector)
% q_pt: position of a waypoint (vector)
% v_pt: velocity of a waypoint (vector)
% t: sample time (scaler)
%
% Output:
% qd: position at time t
% vd: velocity at time t
% ad: accleration at time t
%
% Example: suppose that we want to create a cubic polynomial trajectory
% that passes [0, pi/2, pi/4] with velocity [0, 0, 0], at time [0, 1, 2].
% and we want to know the position, velocity, and accleration of the
% trajectory at t = 1.5, then use
%
% [qd, vd, ad] = cubicpoly([0, 1, 2], [0, pi/2, pi/4], [0, 0, 0], 1.5)
%
% Note: Here, we are trying to create a cubic trajectory of the form
%
% q(t) = a_0 + a_1*t + a_2*t^2 + a_3*t^3,
%
% which corresponds to four constraints that the trajectory must satisfy
% (start/end positions and velocities). The velocity is
%
% v(t) = dq(t)/dt = a_1 + 2*a_2*t + 3*a_3*t^2.
%
% With the constraints, we have four equations
%
% q_0 = a_0 + a_1*t_0 + a_2*t_0^2 + a_3*t_0^3
% v_0 = a_1 + 2*a_2*t_0 + 3*a_3*t_0^2
% q_f = a_0 + a_1*t_f + a_2*t_f^2 + a_3*t_f^3
% v_f = a_1 + 2*a_2*t_f + 3*a_3*t_f^2
%
% As long as (t_f - t_0) is non-zero, there always exists a unique solution.
%
% Reference: Robot modeling and control, Mark W. Spong, Section 7.5
%
%%
function [qd, vd, ad] = cubicpoly(t_pt, q_pt, v_pt, t)
if (length(t_pt) < 2 || length(q_pt) < 2 || length(v_pt) < 2)
error("t_pt, q_pt, v_pt should be vectors");
elseif (length(t) >= 2)
error("t should be a scalar")
end
i = find(t < t_pt, 1) - 1;
if t == t_pt(end)
i = length(t_pt) - 1;
end
t0 = t_pt(i); tf = t_pt(i+1);
q0 = q_pt(i); qf = q_pt(i+1);
v0 = v_pt(i); vf = v_pt(i+1);
T = [1, t0, t0^2, t0^3;
0, 1, 2*t0, 3*t0^2;
1, tf, tf^2, tf^3;
0, 1, 2*tf, 3*tf^2];
b = [q0; v0; qf; vf];
a = T\b;
qd = a'*[1, t, t^2, t^3]';
vd = a'*[0, 1, 2*t, 3*t^2]';
ad = a'*[0, 0, 2, 6*t]';
end