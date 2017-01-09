function [x_sim] = sim_rk4(t, x0, u, param)

x_sim= zeros(3, size(t, 2));
deltaT= t(2)-t(1);
x_sim(:, 1)= x0;
    for i=1:size(u,1)
        x_sim(:,i+1)= rk4_step(deltaT, x_sim(:,i), u(i, :),@robot_ode, param);
    end
end