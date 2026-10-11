%Tepee Simulation Code
%clear stuff
close all
clear
clc
%define knowns
r2 = 10; % tepee length(ft)
r3 = r2;
t1 =0;
tol = 0.001;
%define inputs
t2start =80*(pi/180); %theta(rad)
t2end = 0*(pi/180);
n = 30; %number of frames to count
t2 = linspace(t2start, t2end, n)';
t2d = 1; %velocity
t2dd = 0; %acceleration
%preallocate arrays
r1 = zeros(n,1);
t3 = zeros(n,1);
r1d = zeros(n,1);
t3d = zeros(n,1);
r1dd = zeros(n,1);
t3dd = zeros(n,1);
% initial guess
r1g = 0;
t3g = pi/2 +0.1;
guess = [r1g; t3g];
%calculate unknowns
for ct = 1:n

 %calc vector loop position scalars
 x = r1g- r2*cos(t2(ct)) + r3*cos(t3g);
 y = - r2*sin(t2(ct)) + r3*sin(t3g);
 f = [x; y];

 %calc error
 err = norm(f);

 %guess and check
 while err > tol

 %jacobian

 J = [1, -r3*sin(t3g);...
 0, r3*cos(t3g)];

 %new guess
 guess = guess - J\f;
 r1g = guess(1);
 t3g = guess(2);

 % re-clac position scalars
 x = r1g- r2*cos(t2(ct)) + r3*cos(t3g);
 y = - r2*sin(t2(ct)) + r3*sin(t3g);
 f = [x; y];

%re-clac error
 err = norm(f);
 end

 %save answers
 r1(ct) = r1g;
 t3(ct) = t3g;
%calculate velocity and acceleration
 %jacobian
 J = [1, -r3*sin(t3(ct));...
 0, r3*cos(t3(ct))];

 %velocity
 bv = [-r2*sin(t2(ct))*t2d; r2*cos(t2(ct))*t2d];
 thetadot = J\bv;
 r1d(ct) = thetadot(1);
 t3d(ct) = thetadot(2);

 %acceleration
 ba(1) = -r2*cos(t2(ct))*t2d^2-r2*sin(t2(ct))*t2dd+r3*cos(t3(ct))*t3d(ct)^2;
 ba(2) = -r2*sin(t2(ct))*t2d^2-r2*cos(t2(ct))*t2dd+r3*sin(t3(ct))*t3d(ct)^2;
 thetaddot = inv(J).*ba;
 r1dd(ct) = thetaddot(1);
 t3dd(ct) = thetaddot(2);

end
for ct = 1:n
%calculate joint location
p0x = 0;
p0y = 0;
p23x = r2*cos(t2(ct));
p23y = r2*sin(t2(ct));
p13x = r1(ct);
p13y = 0;
%calculation lines
leg2x = [p0x p23x];
leg2y = [p0y p23y];
leg3x = [p13x p23x];
leg3y = [p13y p23y];
floorx = [p0x p13x];
floory = [p0y p13y];
%plot mechanism
figure(1)
clf
hold on
plot(leg2x, leg2y, 'b')
plot(leg3x, leg3y, 'r')
plot(floorx, floory, 'g')
xlim([0 20])
ylim([0 15])
pause(0.1)
end
%%Static Plots
% Position
Pos = [r1, t3];
figure(2)
hold on
plot(t2,Pos, 'LineWidth', 2)
xlabel('\theta_2 (rad)')
ylabel('positions')
name = legend('${r}_{1}$ (m)','${\theta}_{3}$ (rad)');
set(name,'Interpreter','latex');
title('Joint Positions')
pause(0.5)
%Velocity
Vel = [r1d , t3d];
figure(3)
hold on
plot(t2,Vel,'LineWidth',2)
xlabel('\theta_2 (rad)')
ylabel('velocities')
name = legend('$\dot{r}_{1}$ (m/s)','$\dot{\theta}_{3}$ (rad/s)$');
set(name,'Interpreter','latex');
title('Joint Velocities')
pause(0.5)
%acceleration
Accel = [r1dd, t3dd];
figure(4)
hold on
plot(t2,Accel, 'LineWidth', 2)
xlabel('\theta_2 (rad)')
ylabel('acceleration')
name = legend('$\ddot{r}_{1}$ (m/s^2)','$\ddot{\theta}_{3}$ (rad/s^2)$');
set(name,'Interpreter','latex');
title('Joint Accelerations')
pause(0.5)
