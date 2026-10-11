function out = run_original_submission()
% RUN_ORIGINAL_SUBMISSION  Runs the course script original_submission/tepee_simulation.m (ME 4504, Group 4)
% up to its animation loop and returns the computed arrays, without changing the script itself.
src = fileread(fullfile(fileparts(mfilename('fullpath')), 'original_submission', 'tepee_simulation.m'));
k = strfind(src, 'for ct = 1:n');                    % the second loop is the animation
src = src(1:k(2) - 1);
src = regexprep(src, '^\s*(close all|clear|clc)\s*$', '', 'lineanchors');
eval(src);
out = struct('theta2', t2, 'r1', r1, 'theta3', t3, 'r1d', r1d, 'theta3d', t3d, 'r1dd', r1dd, 'theta3dd', t3dd);
end
