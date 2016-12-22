function [ theta_ml_new , Q_new] = RLS(theta_ml, Q, phi_new, y_new, alpha)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Q_new = alpha * Q + phi_new * phi_new';

control_dim = size(theta_ml,2);

theta_ml_new = zeros(size(theta_ml));

for c = 1:control_dim
    theta_ml_new(:,c) = theta_ml(:,c) + inv(Q_new) * phi_new * (y_new(c) - phi_new' * theta_ml(:,c));
end

end

