function out = tepee_kinematics(r2, r3, t2, t2d, t2dd, tol, q0)
% TEPEE_KINEMATICS  Position, velocity and acceleration of the tepee mechanism
% (a slider-crank whose slider runs along the line through the crank pivot).
%
%   out = tepee_kinematics(r2, r3, t2, t2d, t2dd, tol, q0)
%
%   Inputs   r2, r3   lengths of link 2 (crank) and link 3 (coupler)
%            t2       input angles of link 2 (rad), column vector, solved in order
%            t2d      angular velocity of link 2 (rad/s)
%            t2dd     angular acceleration of link 2 (rad/s^2)
%            tol      Newton stopping tolerance on the norm of the loop residual (default 1e-3)
%            q0       optional initial guess [r1; t3] for the first angle; it selects the assembly
%                     mode (default [0; pi/2 + 0.1], the course guess, which gives t3 = pi - t2)
%   Outputs  out.theta2, out.r1, out.theta3, out.r1d, out.theta3d, out.r1dd, out.theta3dd
%
%   Vector loop (x and y components):
%       r1 - r2*cos(t2) + r3*cos(t3) = 0
%          - r2*sin(t2) + r3*sin(t3) = 0
%   Unknowns q = [r1; t3] are found with Newton's method, warm-started from the previous
%   angle. Velocity and acceleration come from differentiating the loop equations:
%       J*qd  = [ -r2*sin(t2)*t2d ;  r2*cos(t2)*t2d ]
%       J*qdd = [ -r2*cos(t2)*t2d^2 - r2*sin(t2)*t2dd + r3*cos(t3)*t3d^2 ;
%                 -r2*sin(t2)*t2d^2 + r2*cos(t2)*t2dd + r3*sin(t3)*t3d^2 ]
%   with J = [1, -r3*sin(t3); 0, r3*cos(t3)].
%
%   Adapted from the ME 4504 Group 4 course code (see original_submission/): the matrix
%   solve J\ba replaces the element-wise product inv(J).*ba, and r2 ~= r3 and t2dd ~= 0 are handled.
%   J is singular when cos(t3) = 0 (t2 = 90 deg for r2 = r3), so the iteration count is capped.

if nargin < 6, tol = 1e-3; end
if nargin < 7, q0 = [0; pi/2 + 0.1]; end
n = numel(t2);
r1 = zeros(n,1); t3 = zeros(n,1); r1d = zeros(n,1); t3d = zeros(n,1); r1dd = zeros(n,1); t3dd = zeros(n,1);
res = @(q, th) [q(1) - r2*cos(th) + r3*cos(q(2)); -r2*sin(th) + r3*sin(q(2))];
guess = q0(:);
for ct = 1:n
    f = res(guess, t2(ct));  it = 0;
    while norm(f) > tol && it < 50
        J = [1, -r3*sin(guess(2)); 0, r3*cos(guess(2))];
        guess = guess - J\f;
        f = res(guess, t2(ct));  it = it + 1;
    end
    r1(ct) = guess(1);  t3(ct) = guess(2);
    J  = [1, -r3*sin(t3(ct)); 0, r3*cos(t3(ct))];
    bv = [-r2*sin(t2(ct))*t2d; r2*cos(t2(ct))*t2d];
    qd = J\bv;   r1d(ct) = qd(1);   t3d(ct) = qd(2);
    ba = [-r2*cos(t2(ct))*t2d^2 - r2*sin(t2(ct))*t2dd + r3*cos(t3(ct))*t3d(ct)^2; ...
          -r2*sin(t2(ct))*t2d^2 + r2*cos(t2(ct))*t2dd + r3*sin(t3(ct))*t3d(ct)^2];
    qdd = J\ba;  r1dd(ct) = qdd(1);  t3dd(ct) = qdd(2);
end
out = struct('theta2', t2, 'r1', r1, 'theta3', t3, 'r1d', r1d, 'theta3d', t3d, 'r1dd', r1dd, 'theta3dd', t3dd);
end
