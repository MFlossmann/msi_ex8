%MSI Ex 6

%Hendrik Vloet, Mn.: 4324249
%Michael Floßmann 4348852
%Stephan Schraivogel, Mn.: 4318010

clear all; close all; clc;

load('exercise8_data.mat')

%%

grade = 4;

d = grade + 1;
control_dim = size(u,2);

Phi = zeros(size(XYm,1),d,control_dim);

t = linspace(1,size(XYm,1),size(XYm,1));

for i = 1:d
    for j = 1:control_dim
        Phi(:,i,j) = t(:).^(d - i);
    end
end


theta = zeros(d,control_dim);
for i = 1:control_dim
    theta(:,i) = pinv(Phi(:,:,i))*XYm(:,i);
end

fit = @(k) theta'*[k.^4;k.^3;k.^2;k;ones(1,size(k,2))];
fitted_pos = fit(linspace(1,80))';
robot_2D = figure;
hold on;
plot(XYm(:,1),XYm(:,2),'o-')
plot(fitted_pos(:,1),fitted_pos(:,2),'x-')
legend('True position','LLS estimate')
title('2D position and estimate')
hold off;

robot_x = figure;
hold on;
plot(t,XYm(:,1),'o-')
plot(linspace(1,80),fitted_pos(:,1),'x-')
legend('True position','LLS estimate')
title('x position and estimate')
hold off;

robot_y = figure;
hold on;
plot(t,XYm(:,2),'o-')
plot(linspace(1,80),fitted_pos(:,2),'x-')
legend('True position','LLS estimate')
title('x position and estimate')
nhold off;