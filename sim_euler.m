function [ x_sim ] = sim_euler( t, x0, u, param )
%sim_euler Simulates the path of a robot, given the initial position and
%control variable u

N = size(u,1);
m = size(x0,1);

x_sim = zeros(m, N + 1);
x_sim(:,1) = x0;
deltaT = t/(N-1);
for i = 2:N + 1
    x_sim(:,i) = euler_step(deltaT, x_sim(:,i-1), u(i-1,:), @robot_ode, param);
end

end

