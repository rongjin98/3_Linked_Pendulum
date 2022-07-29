function [x,v,a] = SinusoidalSwingUp(amplitude,total_t,current_t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

x = amplitude*sin(current_t/total_t*2*pi);
v = amplitude*cos(current_t/total_t*2*pi)*2*pi;
a = -amplitude*sin(current_t/total_t*2*pi)*(2*pi)^2;
if current_t > total_t
    x = 0;
    v = 0;
    a = 0;
end
end

