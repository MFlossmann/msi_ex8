function [xdot] = robot_ode(t, x, u, param)

%define state space model x_sp for 2D-robot
%velocity (u= [omega_L omega_R]')
v= @(u) (u(1)*param(1)+u(2)*param(2))/2;
beta= x(3);
beta_new= @(u) (u(1)*param(1)-u(2)*param(2))/param(3);

x_pos= @(u) v(u)*cos(beta);
y_pos= @(u) v(u)*sin(beta);

x_sp_fcn= @(u) [x_pos(u); y_pos(u); beta_new(u)];

xdot= x_sp_fcn(u);
    
end