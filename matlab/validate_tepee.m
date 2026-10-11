% VALIDATE_TEPEE  Checks tepee_kinematics against (1) the closed-form solution for r2 = r3 and
% (2) central finite differences of the position for r2 ~= r3 with a nonzero input acceleration.
% Run from the matlab/ folder:  validate_tepee
tol = 1e-12;
% ---- 1. symmetric tepee: r1 = 2 r2 cos(t2), t3 = pi - t2 ----
r2 = 10;  w = 3;                                        % rad/s
t2 = linspace(80*pi/180, 0, 61)';
o = tepee_kinematics(r2, r2, t2, w, 0, tol);
e1 = max(abs(o.r1 - 2*r2*cos(t2)));
e2 = max(abs(o.r1d + 2*r2*sin(t2)*w));
e3 = max(abs(o.r1dd + 2*r2*cos(t2)*w^2));
e4 = max(abs(o.theta3 - (pi - t2)));
fprintf('symmetric  : max |error|  r1 %.1e   r1_dot %.1e   r1_ddot %.1e   theta3 %.1e\n', e1, e2, e3, e4);
assert(max([e1 e2 e3 e4]) < 1e-9);

% ---- 2. unequal links, nonzero angular acceleration: finite differences of r1(t) ----
r2 = 10;  r3 = 12;  w = 2;  alpha = 3;  h = 2e-4;  worst = [0 0 0];
for th0 = (70:-10:10)*pi/180
    at = @(s) tepee_kinematics(r2, r3, th0 + w*s + 0.5*alpha*s^2, w, alpha, tol, [0; pi - th0]);   % angle at time offset s
    a0 = at(0);  ap = at(h);  am = at(-h);
    v_fd = (ap.r1 - am.r1)/(2*h);
    a_fd = (ap.r1 - 2*a0.r1 + am.r1)/h^2;
    w3_fd = (ap.theta3 - am.theta3)/(2*h);
    worst = max(worst, [abs(a0.r1d - v_fd), abs(a0.r1dd - a_fd), abs(a0.theta3d - w3_fd)]);
end
fprintf('r3 = 1.2*r2 : max |code - finite difference|   r1_dot %.1e   r1_ddot %.1e   theta3_dot %.1e\n', worst);
assert(worst(1) < 1e-4 && worst(2) < 1e-3 && worst(3) < 1e-5);   % wrong formulas would miss by O(1-10)
disp('all checks passed');
