function [x,v,a] = CubicPolyTrajectory(xi,xf,vi,vf,total_t)
%Generate a joint space cubic ploynomial trajectory for a 2R manipulator 
% Frequency of 100 Hz   

t = 0:0.01:tf; % Time sampling 100hz

x_i = xi(1);
y_i = xi(2);

x_f = xf(1);
x_f = xf(2);

x_dot_i = vi(1);
y_dot_i = vi(2);

x_dot_f = vf(1);
y_dot_f = vf(2);

x_traj = cubic(x_i,x_f,x_dot_i,x_dot_f,total_t);
y_traj = cubic(y_i,y_f,y_dot_i,y_dot_f,total_t);
end

