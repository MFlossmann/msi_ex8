function [ x_next ] = euler_step( deltaT, x0, u, ode, param )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

x_next = x0 + deltaT*ode(deltaT, x0, u, param);
x_next(3) = wrapToPi(x_next(3));

end

