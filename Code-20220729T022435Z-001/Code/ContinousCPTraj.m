function [x,v,a] = ContinousCPTraj(xi,xf,vi,vf,t0,total_t,current_t)
%Generate a joint space cubic ploynomial trajectory for a 2R manipulator 
%  In continous time

x_i = xi(1);
y_i = xi(2);

x_f = xf(1);
y_f = xf(2);

x_dot_i = vi(1);
y_dot_i = vi(2);

x_dot_f = vf(1);
y_dot_f = vf(2);

%x_traj = cubic(x_i,x_f,x_dot_i,x_dot_f,total_t);
%y_traj = cubic(y_i,y_f,y_dot_i,y_dot_f,total_t);
if current_t > (t0+total_t)
    x = [x_f; y_f];
    v = [0;0];
    a = [0;0];
    return
end
    

tf = t0+total_t;
t = current_t;

%% Calculate the X trajecotry parameters
Mx = [1 t0 t0^2 t0^3;
     0 1  2*t0 3*t0^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2;];
 
Xx = [x_i x_dot_i x_f x_dot_f]';

datax = Mx\Xx;
a0x = datax(1);
a1x = datax(2);
a2x = datax(3);
a3x = datax(4);

%% Calculate the Y trajectory parameters
My = [1 t0 t0^2 t0^3;
     0 1  2*t0 3*t0^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2;];
 
Xy = [y_i y_dot_i y_f y_dot_f]';

datay = My\Xy;
a0y = datay(1);
a1y = datay(2);
a2y = datay(3);
a3y = datay(4);
    
qx = a0x + a1x*t + a2x*t^2 + a3x*t^3; 
q_dotx = a1x + a2x*2*t + a3x*3*t^2;
q_ddotx = a2x*2 + a3x*6*t;
    
qy = a0y + a1y*t + a2y*t^2 + a3y*t^3; 
q_doty = a1y + a2y*2*t + a3y*3*t^2;
q_ddoty = a2y*2 + a3y*6*t;

x = [qx; qy];
v = [q_dotx; q_doty];
a = [q_ddotx; q_ddoty];

    
end
