function [x_sim]=  sim_euler(t, x0, u, param)
%output y= [ x y]'
% t must be integer
x_sim= zeros(3, size(t, 2));
deltaT= t(2)-t(1);
%h = deltaT/M;
h= 0.2;
M= deltaT/h;
x= zeros(3, M);
x_sim(:, 1)= x0;
    for i=1:t(end)-1
        deltaT= t(i+1)-t(i);
        x(:,1)= 0;
        for j=1:M-1
            x(:,j+1)= h*euler_step(deltaT, x(:,j), u(i, :),@robot_ode, param);
        end
        x_sim(:, i+1)= sum(x,2) + x_sim(:, i);
    end

end