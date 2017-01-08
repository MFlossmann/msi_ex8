%MSI Ex 6

%Hendrik Vloet, Mn.: 4324249
%Michael Floßmann 4348852
%Stephan Schraivogel, Mn.: 4318010

clear all; close all; clc;

load('exercise8_data.mat')

%% 1.a)

grade = 4;

d = grade + 1;
control_dim = size(u,2);

N = size(XYm, 1);

Phi = zeros(N,d);

t = linspace(1,N,N);

for i = 1:d
    Phi(:,i) = t(:).^(d - i);
end

theta = zeros(d,control_dim);
for i = 1:control_dim
    theta(:,i) = pinv(Phi)*XYm(:,i);
end

fit = @(k,theta) theta'*[k.^4;k.^3;k.^2;k;ones(1,size(k,2))];
lls_fit = fit(linspace(1,80),theta)';

robot_2D = figure;
hold on;
plot(XYm(:,1),XYm(:,2),'o', 'DisplayName', 'True positions')
plot(lls_fit(:,1),lls_fit(:,2),'-', 'DisplayName', 'LLS fit')
%legend('show');
title('2D position and estimate')
hold off;

rob_x = figure;
hold on;
plot(t,XYm(:,1),'o', 'DisplayName', 'True positions')
plot(linspace(1,80),lls_fit(:,1),'-', 'DisplayName', 'LLS fit')
%legend('show');
title('x position and estimate')
hold off;

rob_y = figure;
hold on;
plot(t,XYm(:,2),'o', 'DisplayName', 'True positions')
plot(linspace(1,80),lls_fit(:,2),'-', 'DisplayName', 'LLS fit')
%legend('show');
title('x position and estimate')
hold off;

%% 1.b)

theta_RLS = ones(d,control_dim);
Q = eye(d);
rls_fit = zeros(N,control_dim);

for i = 1:N
    [theta_RLS, Q] = RLS(theta_RLS, Q, Phi(i,:,:)', XYm(i,:), 1);
    rls_fit(i,:) = fit(i,theta_RLS)';
end

% only was interested in the end
% rls_fit = fit(linspace(1,80),theta_RLS)';

figure(robot_2D);
hold on;
plot(rls_fit(:,1),rls_fit(:,2), '-', 'DisplayName', 'RLS fit')
%legend('show');
hold off;

figure(rob_x);
hold on;
plot(1:size(rls_fit,1),rls_fit(:,1),'-', 'DisplayName', 'RLS fit')
%legend('show');
hold off;

figure(rob_y);
hold on;
plot(1:size(rls_fit,1),rls_fit(:,2),'-', 'DisplayName', 'RLS fit')
%legend('show');
hold off;

%% 1.c)

% jagged?

theta_RLS = zeros(d,control_dim);
Q = eye(d);
rls_fit = zeros(N,control_dim);
for i = 1:N
    [theta_RLS, Q] = RLS(theta_RLS, Q, Phi(i,:,:)', XYm(i,:), 0.5);
    rls_fit(i,:) = fit(i,theta_RLS)';
end

figure(robot_2D);
hold on;
plot(rls_fit(:,1),rls_fit(:,2), '--', 'DisplayName', 'RLS jagged')
%legend('show');
hold off;

figure(rob_x);
hold on;
plot(1:size(rls_fit, 1),rls_fit(:,1),'--', 'DisplayName', 'RLS_\alpha fit')
%legend('show');
hold off;

figure(rob_y);
hold on;
plot(1:size(rls_fit, 1),rls_fit(:,2),'--', 'DisplayName', 'RLS_\alpha fit')
%legend('show');
hold off;

%% smooth?
theta_RLS = zeros(d,control_dim);
Q = eye(d);
rls_fit = zeros(N,control_dim);
for i = 1:N
    [theta_RLS, Q] = RLS(theta_RLS, Q, Phi(i,:,:)', XYm(i,:), 0.85);
    rls_fit(i,:) = fit(i,theta_RLS)';
end

figure(robot_2D);
hold on;
plot(rls_fit(:,1),rls_fit(:,2), '--', 'DisplayName', 'RLS smooth')
legend('show');
hold off;

figure(rob_x);
hold on;
plot(1:size(rls_fit, 1),rls_fit(:,1),'--', 'DisplayName', 'RLS smooth')
legend('show');
hold off;

figure(rob_y);
hold on;
plot(1:size(rls_fit, 1),rls_fit(:,2),'--', 'DisplayName', 'RLS smooth')
legend('show');
hold off;

%% 1.d
% TODO: Implement proper confidence intervals

theta_RLS = zeros(d,control_dim);
Q = eye(d);
% osa = one step ahead
sigma_theta = eye(d);
sigma_theta(:,:,2) = eye(d);
% sigma_osa can be assumed to be diagonal, since x and y are
% assumed to be independent
sigma_osa = zeros(N-1,control_dim);
osa_fit = zeros(N,control_dim);

for i = 1:N
    % extrapolate
    osa_fit(i,:) = Phi(i,:)*theta_RLS;
    
    if i < N
        [theta_RLS, Q] = RLS(theta_RLS, Q, Phi(i+1,:)', XYm(i+1,:), 0.85);
    end
    
    % calculate sigmas for theta and one step ahead estimates
    for j = 1:control_dim
        sigma_theta(:,:,j) =(norm(XYm(1:i,j) - Phi(1:i,:) * theta_RLS(:,j))/(N-d))* ...
                                inv(Phi(1:i,:)'*Phi(1:i,:));
            
        sigma_osa(i,j) = Phi(i+1,:)*sigma_theta(:,:,1)*Phi(i+1,:)';
    end
end

figure(rob_x);
hold on;
plot(1:size(osa_fit, 1),osa_fit(:,1),'r-', 'DisplayName', 'OSA extrap.')
legend('show');
hold off;

figure(robot_2D);
hold on;
plot(osa_fit(:,1),osa_fit(:,2), 'r-', 'DisplayName', 'OSA extrap.')
legend('show');
hold off;
pp
figure(rob_y);
hold on;
plot(1:size(osa_fit, 1),osa_fit(:,2),'r-', 'DisplayName', 'OSA extrap.')
legend('show');
hold off;

%% 2.
clear all;

load('exercise8_data.mat');

param = [0.2; 0.2; 0.6];
x0 = [0;0;0];

x_euler = sim_euler(size(u,1), x0, u, param);
x_rk4 = sim_rk4(size(u,1), x0, u, param);

[bar_euler, foo_euler] = pol2cart(x_euler(3,:),...
                                  ones(1,size(x_euler,2))*0.03);
[bar_rk4, foo_rk4] = pol2cart(x_rk4(3,:),...
                              ones(1,size(x_rk4,2))*0.02);
                  
sim_fig = figure;
hold on;
%plot(x_eulerim(2,:),x_eulerim(1,:),'x');
quiver(x_euler(2,:),x_euler(1,:),...
       foo_euler, bar_euler, 0, 'LineWidth', 1.5);
quiver(x_rk4(2,:), x_rk4(1,:),...
       foo_rk4, bar_rk4, 0, 'r', 'LineWidth', 1.5);
hold off;

%% horsin around
% u: one wheel has constant speed, one wheel goes in a sine. This should
% result in a wavelike movement

%straight line up, slowing down constantly, quite boring
u(:,1) = linspace(1,0,90)';
u(:,2) = u(:,1);
x_euler = sim_euler(size(u,1), x0, u, param);
x_rk4 = sim_rk4(size(u,1), x0, u, param);
[bar_euler, foo_euler] = pol2cart(x_euler(3,:),...
                      ones(1,size(x_euler,2))*0.03);
[bar_rk4, foo_rk4] = pol2cart(x_rk4(3,:),...
                              ones(1,size(x_rk4,2))*0.02);                  
figure;                  
quiver(x_euler(2,:),x_euler(1,:),...
       foo_euler, bar_euler, 0, 'LineWidth', 1.5);
quiver(x_rk4(2,:), x_rk4(1,:),...
       foo_rk4, bar_rk4, 0, 'r', 'LineWidth', 1.5);   
title('Simple (boring) line');

%% circle
u(:,1) = 0.5*ones(90,1);
u(:,2) = 1.*ones(90,1);
x_euler = sim_euler(size(u,1), x0, u, param);
x_rk4 = sim_rk4(size(u,1), x0, u, param);
[bar_euler, foo_euler] = pol2cart(x_euler(3,:),...
                                  ones(1,size(x_euler,2))*0.03);
[bar_rk4, foo_rk4] = pol2cart(x_rk4(3,:),...
                              ones(1,size(x_rk4,2))*0.05);
figure;            
quiver(x_euler(2,:),x_euler(1,:),...
       foo_euler, bar_euler, 0, 'LineWidth', 1.5);
quiver(x_rk4(2,:), x_rk4(1,:),...
       foo_rk4, bar_rk4, 0, 'r', 'LineWidth', 1.5);   
title('Circle');

%% squiggles!
u(:,1) = 1.*ones(90,1);
for i = 1:90
    u(i,1) = 3.+((-1)^floor(i/2))*mod(i,2);
    u(i,2) = 3.-((-1)^floor(i/2))*mod(i,2);
end
x_euler = sim_euler(size(u,1), x0, u, param);
x_rk4 = sim_rk4(size(u,1), x0, u, param);
[bar_euler, foo_euler] = pol2cart(x_euler(3,:),...
                      ones(1,size(x_euler,2))*0.3);
[bar_rk4, foo_rk4] = pol2cart(x_rk4(3,:),...
                              ones(1,size(x_rk4,2))*0.);                  
figure;                  
quiver(x_euler(2,:),x_euler(1,:),...
       foo_euler, bar_euler, 0, 'LineWidth', 1.5);
quiver(x_rk4(2,:), x_rk4(1,:),...
       foo_rk4, bar_rk4, 0, 'r', 'LineWidth', 1.5);   
title('Squiggles!');

%%
clear u;
u(:,1) = 3*cos(linspace(0,30*pi,180))';
u(:,2) = 3*sin(linspace(0,30*pi,180))';

x_euler = sim_euler(size(u,1), x0, u, param);
x_rk4 = sim_rk4(size(u,1), x0, u, param);
[bar_euler, foo_euler] = pol2cart(x_euler(3,:),...
                      ones(1,size(x_euler,2))*0.1);
[bar_rk4, foo_rk4] = pol2cart(x_rk4(3,:),...
                              ones(1,size(x_rk4,2))*0.1);                  
figure;                  
quiver(x_euler(2,:),x_euler(1,:),...
       foo_euler, bar_euler, 0, 'LineWidth', 1.5);
quiver(x_rk4(2,:), x_rk4(1,:),...
       foo_rk4, bar_rk4, 0, 'r', 'LineWidth', 2);   
title('Different squiggles');

