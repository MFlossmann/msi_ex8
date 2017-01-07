function [x_next]= euler_step(deltaT, x0, u, ode, param)

%Script Diehl p. 57 bottom

x_next= x0 + deltaT*ode(deltaT, x0, u, param);

end
