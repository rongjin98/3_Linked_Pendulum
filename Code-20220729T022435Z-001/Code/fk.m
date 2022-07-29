function Xe = fk(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
params 

theta_1 = q(1);
theta_2 = q(2);

X = a_1*cos(theta_1) + a_2*cos(theta_1+theta_2);
Y = a_1*sin(theta_1) + a_2*sin(theta_1+theta_2);

Xe = [X Y]';
end

