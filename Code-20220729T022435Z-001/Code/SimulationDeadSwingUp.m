close all
clear all 
clc

tic
params;

% Initial conditions
disturbance = 0.0;
qi = ik(-1.5,0.75)
theta_1_0 = qi(1);
theta_2_0 = qi(2);
theta_3_0 = wrapToPi(-(qi(1)+qi(2)+pi/2+disturbance));
theta_dot_1_0 = 0;
theta_dot_2_0 = 0;
theta_dot_3_0 = 0;
 
% theta_1_0 = 1.0399;  
% theta_2_0 = 5.1307;
% theta_3_0 = 7.8842;
% theta_dot_1_0 = -0.0001;
% theta_dot_2_0 = 0;
% theta_dot_3_0 = 0.1634;
tspan = [0 3];

% ODE45 settings
options = odeset('RelTol',1e-3,'AbsTol',1e-5);
%options = odeset('reltol', 1E-7);
z_0 = [theta_1_0; theta_2_0; theta_3_0; theta_dot_1_0; theta_dot_2_0; theta_dot_3_0];

B_mat = load('Mass_matrix.mat');
B_f = B_mat.Mass_matrix;
N_mat = load('N_term.mat');
N_f = N_mat.N_matrix;

[time, z] = ode45(@(t,y) manipulator(t,y,B_f,N_f), tspan, z_0, options);

toc

%%
% Get varibles from simulation
theta_1 = z(:,1);
theta_2 = z(:,2);
theta_3 = z(:,3);
theta_1_dot = z(:,4);
theta_2_dot = z(:,5);
theta_3_dot = z(:,6);

Xd = zeros(2,length(time));
Xd_dot = zeros(2,length(time));
Xd_ddot = zeros(2,length(time));

q = [theta_1 theta_2 theta_3]';
q_dot = [theta_1_dot theta_2_dot theta_3_dot]';

phi = wrapToPi(-q(1,:)-q(2,:)-q(3,:)+pi/2);

for i = 1:length(time)
    Xe(:,i) = fk(q(1:2,i));
    % Jacobian Analytical
    j11 = -a_2*sin(theta_1(i)+theta_2(i))-a_1*sin(theta_1(i));
    j12 = -a_2*sin(theta_1(i)+theta_2(i));
    j21 = a_2*cos(theta_1(i)+theta_2(i))+a_1*cos(theta_1(i));
    j22 = a_2*cos(theta_1(i)+theta_2(i));
    Ja = [j11 j12; j21 j22];
    Xe_dot(:,i) = Ja*q_dot(1:2,i);
    %[x,v,a] = SinusoidalSwingUp(0.485,0.5,time(i));
    %%%
    Xi = [-1.5; 0.75];
    Xf = Xi + [3.0; 0.0];
    Vi = [0;0];
    Vf = [6.0;0];
    [x,v,a] = ContinousCPTraj(Xi,Xf,Vi,Vf,1.5,time(i));
    %%%
    Xd(:,i) = [x(1) 0.75]';
    Xd_dot(:,i) = [v(1) 0]';
    Xd_ddot(:,i) = [a(1) 0]';
end

hold on 
plot(time,theta_1)
plot(time,theta_2)
plot(time,theta_3)
hold off

figure

subplot(3,1,1)
xlim([0,3])

hold on
plot(time,Xe(1,:))
plot(time,Xd(1,:))
hold off
title("Endeffector Position Swing Up")
ylabel("Position X (m)")
xlabel("Time (sec)")
legend("Actual", "Desired")

subplot(3,1,2)
xlim([0,3])

hold on
plot(time,Xe(2,:))
plot(time,Xd(2,:))
hold off
ylabel("Position Y (m)")
xlabel("Time (sec)")
legend("Actual", "Desired")

%axis([0 tspan(1,2) 0.71 0.79])

subplot(3,1,3)
xlim([0,3])

hold on
plot(time,phi)
yline(0, 'r')
yline(-0.3,'--b');
yline(0.3, '--b');
hold off
ylabel("Angle Phi (rad)")
xlabel("Time (sec)")
legend("Actual", "Desired")

bound  = 0.86;

figure

subplot(4,1,1)
plot(time,Xd(1,:)-Xe(1,:));
title("Error plots for X and Y Position and Velocity")
xlabel("Time (sec)");
ylabel("X Position Error");
xlim([0,bound])


subplot(4,1,2)

plot(time,Xd_dot(1,:)-Xe_dot(1,:));
xlabel("Time (sec)");
ylabel("X Velocity Error");
xlim([0,bound])


subplot(4,1,3)

plot(time,Xd(1,:)-Xe(1,:));
xlabel("Time (sec)");
ylabel("Y Position Error");
xlim([0,bound])

subplot(4,1,4)

plot(time,Xd_dot(2,:)-Xe_dot(2,:));
xlabel("Time (sec)");
ylabel("Y Velocity Error");
xlim([0,bound])


% Calculating link endpoints
L1_x = a_1*cos(theta_1);
L1_y = a_1*sin(theta_1);
L2_x = L1_x + a_2*cos(theta_1+theta_2);
L2_y = L1_y + a_2*sin(theta_1+theta_2);
L3_x = L2_x + a_3*cos(theta_1+theta_2+theta_3);
L3_y = L2_y + a_3*sin(theta_1+theta_2+theta_3);

tempcmd = sprintf('h = figure(''name'',''Part b: Animation of Planar 2R manip (no control torques)'', ''NumberTitle'',''off'');');
eval(tempcmd);
axis([-1.75 1.75 -1 2.5]);
xlabel('x');
ylabel('y');

L1_line = line('xdata', [0, L1_x(1)], 'ydata', [0, L1_y(1)],...
    'linewidth', 3, 'color', 'blue');
L2_line = line('xdata', [L1_x(1), L2_x(1)], 'ydata', [L1_y(1), L2_y(1)],...
    'linewidth', 3, 'color', 'red');
L3_line = line('xdata', [L2_x(1), L3_x(1)], 'ydata', [L2_y(1), L3_y(1)],...
    'linewidth', 3, 'color', 'green');

%%
delay = tspan(1,2)/length(time);
hold on
yline(0.75);
xline(0);
% Draw and redraw the objects with new xdata, ydata
for i=1:length(time)
    pause(delay*0)
    set(L1_line, 'xdata', [      0, L1_x(i)], 'ydata', [      0, L1_y(i)]);
    set(L2_line, 'xdata', [L1_x(i), L2_x(i)], 'ydata', [L1_y(i), L2_y(i)]);
    set(L3_line, 'xdata', [L2_x(i), L3_x(i)], 'ydata', [L2_y(i), L3_y(i)]);
    drawnow;
end

hold off