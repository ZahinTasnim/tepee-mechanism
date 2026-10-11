% TEPEE_ANALYSIS  Complete analysis of the tepee mechanism and every figure in the repository.
%
%   1. runs the course script as submitted and the revised function tepee_kinematics.m
%      for the course inputs (r2 = r3 = 10, theta2 from 80 to 0 deg, 30 frames, w = 1 rad/s)
%   2. compares both with the closed-form solution r1 = 2 r2 cos(theta2)
%   3. extends the analysis to a full crank revolution (assembly modes, singular positions)
%   4. checks the Scotch-yoke equivalence and the straight-line point on the extended coupler
%   5. writes figures/01-03, figures/tepee_motion.gif, matlab/outputs/*.csv, results/summary.txt
%
%   Run from MATLAB or GNU Octave:  tepee_analysis

here = fileparts(mfilename('fullpath'));  root = fileparts(here);
figdir = fullfile(root, 'figures');  resdir = fullfile(root, 'results');  outdir = fullfile(here, 'outputs');
for d = {figdir, resdir, outdir}, if ~exist(d{1}, 'dir'), mkdir(d{1}); end, end
isOctave = exist('OCTAVE_VERSION', 'builtin') > 0;
if isOctave, graphics_toolkit('gnuplot'); end        % headless plotting in Octave
set(0, 'defaultaxesfontsize', 11); set(0, 'defaulttextfontsize', 11);
blue = [0.12 0.37 0.62]; red = [0.75 0.22 0.17]; green = [0.18 0.55 0.34]; orange = [0.83 0.33 0.00]; grey = [0.6 0.6 0.6];

%% 1. course inputs: submitted script and revised function
R = 10;
orig = run_original_submission();
t2 = orig.theta2;
rev = tepee_kinematics(R, R, t2, 1, 0, 1e-10);
cf.r1 = 2*R*cos(t2);  cf.theta3 = pi - t2;  cf.r1d = -2*R*sin(t2);  cf.theta3d = -ones(size(t2));  cf.r1dd = -2*R*cos(t2);
flds = {'r1', 'theta3', 'r1d', 'theta3d', 'r1dd'};
for k = 1:numel(flds)
    errOrig(k) = max(abs(orig.(flds{k}) - cf.(flds{k})));
    errRev(k)  = max(abs(rev.(flds{k})  - cf.(flds{k})));
end
hdr = 'theta2_deg,r1,theta3_rad,r1_dot,theta3_dot,r1_ddot,theta3_ddot';
write_csv(fullfile(outdir, 'original_submission_results.csv'), hdr, ...
    [rad2deg(t2) orig.r1 orig.theta3 orig.r1d orig.theta3d orig.r1dd orig.theta3dd]);
write_csv(fullfile(outdir, 'tepee_results.csv'), hdr, ...
    [rad2deg(t2) rev.r1 rev.theta3 rev.r1d rev.theta3d rev.r1dd rev.theta3dd]);

%% 2. assembly modes: same input angle, two solutions of the loop equations
modeAngles = [30 60 80];
for k = 1:numel(modeAngles)
    th = deg2rad(modeAngles(k));
    a = tepee_kinematics(R, R, th, 1, 0, 1e-13, [0; pi - th + 0.05]);   % tepee configuration
    b = tepee_kinematics(R, R, th, 1, 0, 1e-13, [0; th + 0.05]);        % folded configuration
    modes(k, :) = [modeAngles(k) a.r1 rad2deg(a.theta3) b.r1 rad2deg(b.theta3)];
end

%% 3./4. full revolution, equivalence checks
d = (0:0.5:360)';  th = deg2rad(d);
r1full = 2*R*cos(th);   detJ = R*cos(pi - th);              % det J = r3 cos(theta3) on the tepee branch
P = [R*cos(th) R*sin(th)];  S = [r1full zeros(size(th))];  Q = 2*P - S;
scotchDiff = max(abs(r1full - 20*cos(th)));                 % Scotch yoke with crank radius 20
qxMax = max(abs(Q(:, 1)));
couplerLen = max(abs(sqrt(sum((P - S).^2, 2)) - R));

%% 5a. figure 01: mechanism and course range
f = figure('visible', 'off', 'position', [0 0 1150 440]);
subplot(1, 2, 1); hold on; axis equal; axis off;
t = deg2rad(60); p = R*[cos(t) sin(t)]; s = [2*R*cos(t) 0];
plot([-4 24], [0 0], 'color', [0.8 0.8 0.8], 'linewidth', 8);
plot([0 p(1)], [0 p(2)], 'color', blue, 'linewidth', 3);  plot([p(1) s(1)], [p(2) s(2)], 'color', red, 'linewidth', 3);
rectangle('Position', [s(1)-1.6 -1 3.2 2], 'FaceColor', green, 'EdgeColor', 'k');
plot([0 p(1) s(1)], [0 p(2) 0], 'ko', 'markerfacecolor', 'k', 'markersize', 6);
text(-3.4, -2.2, 'O (fixed pivot)'); text(p(1)-1.0, p(2)+1.0, 'P'); text(s(1)-1.2, -3.0, 'S (slider)');
text(p(1)/2-2.6, p(2)/2+0.4, 'r_2', 'color', blue, 'fontsize', 13); text((p(1)+s(1))/2+0.7, p(2)/2+0.6, 'r_3', 'color', red, 'fontsize', 13);
text(3.4, 0.9, '\theta_2', 'fontsize', 12); text(s(1)+1.8, 1.6, '\theta_3 = \pi - \theta_2', 'color', red);
plot([0 s(1)], [-5 -5], 'k-'); plot([0 0], [-5.6 -4.4], 'k-'); plot([s(1) s(1)], [-5.6 -4.4], 'k-'); text(s(1)/2-3.5, -6.4, 'r_1 = 2 r_2 cos\theta_2');
xlim([-5 24]); ylim([-8 12.5]); title('Tepee mechanism: 4 links, 4 joints, M = 1');
subplot(1, 2, 2); hold on; axis equal; axis off;
plot([0 20], [0 0], 'color', [0.8 0.8 0.8], 'linewidth', 6);
snap = linspace(80, 0, 6); cols = [0.27 0.00 0.33; 0.23 0.32 0.55; 0.13 0.57 0.55; 0.21 0.72 0.47; 0.56 0.84 0.26; 0.99 0.91 0.14];
for k = 1:6
    t = deg2rad(snap(k)); p = R*[cos(t) sin(t)];
    h(k) = plot([0 p(1) 2*R*cos(t)], [0 p(2) 0], '-o', 'color', cols(k, :), 'linewidth', 2, 'markersize', 4, 'markerfacecolor', cols(k, :));
end
legend(h, arrayfun(@(a) sprintf('\\theta_2 = %d deg', round(a)), snap, 'UniformOutput', false), 'location', 'northeast');
title('Course range: \theta_2 from 80 to 0 deg (r_2 = r_3 = 10)');
print(f, fullfile(figdir, '01_mechanism.png'), '-dpngcairo', '-r110'); close(f);

%% 5b. figure 02: position, velocity, acceleration over the course range
deg2 = t2;  f = figure('visible', 'off', 'position', [0 0 760 820]);
pairs = {{'r1', 'theta3', 'r_1', '\theta_3 (rad)', 'Position'}, {'r1d', 'theta3d', 'dr_1/dt', 'd\theta_3/dt (rad/s)', 'Velocity'}, ...
         {'r1dd', 'theta3dd', 'd^2r_1/dt^2', 'd^2\theta_3/dt^2 (rad/s^2)', 'Acceleration'}};
for k = 1:3
    pr = pairs{k}; subplot(3, 1, k); hold on; grid on;
    plot(deg2, orig.(pr{1}), 'color', blue, 'linewidth', 2.2);
    plot(deg2, orig.(pr{2}), 'color', orange, 'linewidth', 2.2);
    plot(deg2, cf.(pr{1}), 'k--', 'linewidth', 1);
    legend({[pr{3} ' (course code)'], [pr{4} ' (course code)'], 'closed form'}, 'location', 'eastoutside');
    title(pr{5}); if k == 3, xlabel('\theta_2 (rad)'); end
end
print(f, fullfile(figdir, '02_kinematics_course_range.png'), '-dpngcairo', '-r110'); close(f);

%% 5c. figure 03: full revolution and Jacobian determinant
f = figure('visible', 'off', 'position', [0 0 780 680]);
subplot(2, 1, 1); hold on; grid on;
fill([0 80 80 0], [-22 -22 22 22], [0.96 0.87 0.68], 'edgecolor', 'none');
plot(d, r1full, 'color', blue, 'linewidth', 2.2); plot(d, zeros(size(d)), '--', 'color', red, 'linewidth', 1.6);
plot([90 270], [0 0], 'ko', 'markerfacecolor', 'k', 'markersize', 6);
legend({'course range (0 to 80 deg)', 'tepee branch: r_1 = 2 r_2 cos\theta_2', 'folded branch: r_1 = 0', 'det J = 0'}, 'location', 'eastoutside');
ylabel('slider position r_1'); xlim([0 360]); ylim([-22 22]); set(gca, 'xtick', 0:60:360); title('Slider position over a full crank revolution');
subplot(2, 1, 2); hold on; grid on;
plot(d, detJ, 'color', green, 'linewidth', 2.2); legend({'det J'}, 'location', 'eastoutside'); plot([90 90], [-11 11], 'k:'); plot([270 270], [-11 11], 'k:');
text(96, -7.5, 'det J = 0 at 90 and 270 deg: slider at the pivot');
xlabel('\theta_2 (deg)'); ylabel('det J = r_3 cos\theta_3'); xlim([0 360]); ylim([-11 11]); set(gca, 'xtick', 0:60:360);
title('Jacobian determinant on the tepee branch');
print(f, fullfile(figdir, '03_full_revolution.png'), '-dpngcairo', '-r110'); close(f);

%% 5d. animation over a full revolution
% Note: Octave writes GIF frames without LZW compression (large file); MATLAB compresses them.
gifFile = fullfile(figdir, 'tepee_motion.gif');  frame = fullfile(tempdir, 'tepee_frame.png');
for k = 0:71
    a = deg2rad(5*k); p = R*[cos(a) sin(a)]; s = 2*R*cos(a);
    f = figure('visible', 'off', 'position', [0 0 560 300]); hold on; axis equal; axis off;
    plot([-23 23], [0 0], 'color', [0.85 0.85 0.85], 'linewidth', 8);
    c = linspace(0, 2*pi, 100); plot(R*cos(c), R*sin(c), '--', 'color', [0.8 0.8 0.8]);
    plot([0 p(1)], [0 p(2)], 'color', blue, 'linewidth', 3); plot([p(1) s], [p(2) 0], 'color', red, 'linewidth', 3);
    rectangle('Position', [s-1.6 -1 3.2 2], 'FaceColor', green, 'EdgeColor', 'k');
    plot([0 p(1) s], [0 p(2) 0], 'ko', 'markerfacecolor', 'k', 'markersize', 5);
    text(-6, 12, sprintf('theta_2 = %3d deg    r_1 = %6.2f', 5*k, s), 'interpreter', 'none');
    xlim([-24 24]); ylim([-13 13.5]);
    print(f, frame, '-dpngcairo', '-r80'); close(f);
    img = imread(frame);
    if isOctave
        [ind, map] = rgb2ind(uint8(round(double(img)/51)*51));   % 6 levels per channel (<= 216 colours)
    else
        [ind, map] = rgb2ind(img, 64);
    end
    if k == 0, imwrite(ind, map, gifFile, 'gif', 'DelayTime', 0.06, 'LoopCount', Inf);
    else, imwrite(ind, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.06); end
end

%% 6. summary
fid = fopen(fullfile(resdir, 'summary.txt'), 'w');
fprintf(fid, 'Tepee mechanism, r2 = r3 = %g, course range theta2 = 80 -> 0 deg, theta2_dot = 1 rad/s\n\n', R);
fprintf(fid, 'Max |result - closed form|        %-10s %-10s %-10s %-10s %-10s\n', flds{:});
fprintf(fid, '  course code (tol 1e-3)          %-10.1e %-10.1e %-10.1e %-10.1e %-10.1e\n', errOrig);
fprintf(fid, '  revised function (tol 1e-10)    %-10.1e %-10.1e %-10.1e %-10.1e %-10.1e\n\n', errRev);
fprintf(fid, 'End points (closed form): theta2 = 80 deg: r1 = %.3f, r1_dot = %.3f, r1_ddot = %.3f\n', 2*R*cosd(80), -2*R*sind(80), -2*R*cosd(80));
fprintf(fid, '                          theta2 =  0 deg: r1 = %.3f, r1_dot = %.3f, r1_ddot = %.3f\n\n', 2*R, 0, -2*R);
fprintf(fid, 'Assembly modes (theta2 [deg], tepee r1, tepee theta3 [deg], folded r1, folded theta3 [deg]):\n');
fprintf(fid, '  %5.0f  %9.4f  %8.2f  %9.1e  %8.2f\n', modes');
fprintf(fid, '\ndet J = r3 cos(theta3) = 0 at theta2 = 90 and 270 deg (slider at the crank pivot)\n');
fprintf(fid, 'Scotch yoke (crank radius 20) vs tepee slider: max |difference| = %.1e\n', scotchDiff);
fprintf(fid, 'Extended coupler point Q = 2P - S: max |x_Q| = %.1e (moves on the line x = 0)\n', qxMax);
fprintf(fid, 'Coupler length check: max |PS - r3| = %.1e\n', couplerLen);
fclose(fid);
type(fullfile(resdir, 'summary.txt'));
