function [ x_next ] = rk4( deltaT, x0, u, ode, param )
%rk4 Computes one Runge-Kutta-method (RK4) step
%   Detailed explanation goes here
k1 = ode(deltaT, x0, u, param);
k2 = ode(deltaT, x0 + k1 * deltaT/2, u, param);
k3 = ode(deltaT, x0 + k2 * deltaT/2, u, param);
k4 = ode(deltaT, x0 + k3 * deltaT, u, param);

x_next = x0 + deltaT * (k1 + k2 + k3 + k4) / 6;

end

