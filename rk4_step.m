function [x_next] = rk4_step(deltaT, x0, u, ode, param)

k= zeros(3,4);

k(:,1)= ode(deltaT, x0, u, param);
k(:,2)= ode(deltaT, x0+(deltaT/2)*k(:,1), u, param);
k(:,3)= ode(deltaT, x0+(deltaT/2)*k(:,2), u, param);
k(:,4)= ode(deltaT, x0+deltaT*k(:,3), u, param);

x_next= x0 + (deltaT/6)*(k(:,1)+2*k(:,2)+2*k(:,3)+k(:,4));

end