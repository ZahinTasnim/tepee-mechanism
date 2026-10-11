function [r1, t3] = solveTeepee(r2, r3, t2)
% Solve unknowns for teepee mechanism
% INPUTS: knowns and mechanism input: r2, r3, t1, t2
% OUTPUTS: unknowns: r1, t3
% solve for theta3
t3 = pi - t2;
% solve for r1 second
r1 = r2*cos(t2) - r3*cos(t3);
end
