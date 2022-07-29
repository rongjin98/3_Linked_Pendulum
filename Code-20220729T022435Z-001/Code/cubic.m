function traj = cubic(q0,qf,v0,vf,t)
% This function generates a time vector of scalars q
% such that is abides by the constraints:
% q0,qf,vi,vf,t
t0 = 0;
tf = t;
time = t0:0.01:tf; % Time sampling 100hz

M = [1 t0 t0^2 t0^3;
     0 1  2*t0 3*t0^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2;];
 
X = [q0 v0 qf vf]';

data = M\X;
a0 = data(1);
a1 = data(2);
a2 = data(3);
a3 = data(4);

for i = 1:(length(time))
    t = (i-1)*0.01; 
    q(i) = a0 + a1*t + a2*t^2 + a3*t^3; 
    q_dot(i) = a1 + a2*2*t + a3*3*t^2;
    q_ddot(i) = a2*2 + a3*6*t;
end
traj = [q',q_dot', q_ddot'];
end
