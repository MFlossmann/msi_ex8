function [xdot] = robot_ode(t, x, u, param)

%define state space model x_sp for 2D-robot
%velocity (u= [omega_L omega_R]')
v= @(u) (u(1)*param(1)+u(2)*param(2))/2;
beta= @(u) (u(1)*param(1)-u(2)*param(2))/param(3);

x_pos= @(u) v(u)*cos(beta(u)');
y_pos= @(u) v(u)*sin(beta(u)');

x_sp_fcn= @(u) [x_pos(u); y_pos(u); beta(u)];

xdot= x_sp_fcn(u);
    
end