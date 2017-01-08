function [ xdot ] = robot_ode( t, x, u, param )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

R_L = param(1);
R_R = param(2);
L = param(3);

om_L = u(1);
om_R = u(2);

v = (om_L*R_L + om_R*R_R)/2;
beta = x(3);

xdot = zeros(3,1);

xdot(1) = v * cos(beta);
xdot(2) = v * sin(beta);
xdot(3) = (om_L*R_L - om_R*R_R)/L;

end

