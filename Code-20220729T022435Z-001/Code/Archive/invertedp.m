function Xp = invertedp(t, x, K)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
K = [-35 -34 -150 -34];

ft = 0.1;
M = 2;
m = 0.1;
l = 0.5;
g = 9.8;
x
U = -K*x;
U = 0;
Xp(1,1) = x(2);
Xp(2,1) = (-m*g*sin(x(3))*cos(x(3))+m*l*x(4)^2*sin(x(3))+ft*m*x(4)*cos(x(3))+U)/(M+(1-cos(x(3))^2)*m);
Xp(3,1) = x(4);
Xp(4,1) = ((M+m)*(g*sin(x(3))-ft*x(4))-(l*m*x(4)^2*sin(x(3))+U)*cos(x(3)))/(l*(M+(1-cos(x(3))^2)));
Xp
end

