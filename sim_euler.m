function [x_sim]=  sim_euler(t, x0, u, param)
%output y= [ x y]'
% t must be integer
x_sim= zeros(3, size(u, 1)+1);
deltaT= t(2)-t(1);
x_sim(:, 1)= x0;
x_next= x0;
    for i=1:size(u,1)
        x_next= euler_step(deltaT, x_next, u(i, :),@robot_ode, param);
        x_sim(:, i+1)= x_next;
    end

end