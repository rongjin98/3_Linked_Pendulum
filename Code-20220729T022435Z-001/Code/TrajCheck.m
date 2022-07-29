clear all
close all
clc


Xi = [0; 0.75];
Xf = Xi + [0.5; 0.50];
Vi = [0;0];
Vf = [0;0];
for i = 0:100
    
    t = (i/100+5);
    [x,v,a] = ContinousCPTraj(Xi,Xf,Vi,Vf,5,1,i/100+5);
    %[x,v,a] = SinusoidalSwingUp(0.48,0.5,t);
    
    xPos(i+1,:) = x';
    xVel(i+1,:) = v';
    xAcc(i+1,:) = a';
end
t = 0:0.01:1;
hold on
title("Sinusoidal swing up trajectory")
plot(t',xPos')
plot(t',xVel')
plot(t',xAcc')
xlabel("Time (sec)")
ylabel("Position, Velocity, Acceleration")
legend("Positon (m)","Velocity (m/s)","Acceleration (m/s^2)");
hold off

