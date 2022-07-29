close all
clear all 
clc

tic
params;

% Initial conditions
disturbance = -0.0;
qi = ik(0,0.75)
theta_1_0 = qi(1);
theta_2_0 = qi(2);
theta_3_0 = wrapToPi(-(qi(1)+qi(2)+pi/2+disturbance));
theta_dot_1_0 = 0;
theta_dot_2_0 = 0;
theta_dot_3_0 = 0;

tspan = [0 10];

% ODE45 settings
options = odeset('RelTol',1e-3,'AbsTol',1e-5);
%options = odeset('reltol', 1E-7);
z_0 = [theta_1_0; theta_2_0; theta_3_0; theta_dot_1_0; theta_dot_2_0; theta_dot_3_0];

B_mat = load('Mass_matrix.mat');
B_f = B_mat.Mass_matrix;
N_mat = load('N_term.mat');
N_f = N_mat.N_matrix;

[time, z] = ode23tb(@(t,y) manipulator(t,y,B_f,N_f), tspan, z_0, options);

toc

%%
% Get varibles from simulation
theta_1 = z(:,1);
theta_2 = z(:,2);
theta_3 = z(:,3);

q = [theta_1 theta_2 theta_3]';
phi = wrapToPi(-q(1,:)-q(2,:)-q(3,:)+pi/2);
for i = 1:length(time)
    Xe(:,i) = fk(q(1:2,i));
end

hold on 
plot(time,theta_1)
plot(time,theta_2)
plot(time,theta_3)
hold off

figure

subplot(3,1,1)
plot(time,Xe(1,:))
title("Endeffector position step response (Balance Controller)")
ylabel("Position X (m)")
xlabel("Time (sec)")

subplot(3,1,2)
plot(time,Xe(2,:))
ylabel("Position Y (m)")
xlabel("Time (sec)")
%axis([0 tspan(1,2) 0.71 0.79])

subplot(3,1,3)
plot(time,phi)
ylabel("Angle Phi (rad)")
xlabel("Time (sec)")

% Calculating link endpoints
L1_x = a_1*cos(theta_1);
L1_y = a_1*sin(theta_1);
L2_x = L1_x + a_2*cos(theta_1+theta_2);
L2_y = L1_y + a_2*sin(theta_1+theta_2);
L3_x = L2_x + a_3*cos(theta_1+theta_2+theta_3);
L3_y = L2_y + a_3*sin(theta_1+theta_2+theta_3);

[L1_x ts] = resample(L1_x,time);
[L1_y ts] = resample(L1_y,time);
[L2_x ts] = resample(L2_x,time);
[L2_y ts] = resample(L2_y,time);
[L3_x ts] = resample(L3_x,time);
[L3_y ts] = resample(L3_y,time);

figure()
axis([-2.5 2.5 -2.5 2.5]);
xlabel('x');
ylabel('y');

L1_line = line('xdata', [0, L1_x(1)], 'ydata', [0, L1_y(1)],...
    'linewidth', 3, 'color', 'blue');
L2_line = line('xdata', [L1_x(1), L2_x(1)], 'ydata', [L1_y(1), L2_y(1)],...
    'linewidth', 3, 'color', 'red');
L3_line = line('xdata', [L2_x(1), L3_x(1)], 'ydata', [L2_y(1), L3_y(1)],...
    'linewidth', 3, 'color', 'green');

%%
delay = tspan(1,2)/length(ts);
hold on
%yline(0.75);
%xline(0);
grid on
v = VideoWriter('Output.avi');
open(v);
% Draw and redraw the objects with new xdata, ydata
for i=1:length(ts)
    pause(delay)
    set(L1_line, 'xdata', [      0, L1_x(i)], 'ydata', [      0, L1_y(i)]);
    set(L2_line, 'xdata', [L1_x(i), L2_x(i)], 'ydata', [L1_y(i), L2_y(i)]);
    set(L3_line, 'xdata', [L2_x(i), L3_x(i)], 'ydata', [L2_y(i), L3_y(i)]);
    frame = getframe(gcf);
    writeVideo(v,frame);
    drawnow;
end

hold off
close(v)