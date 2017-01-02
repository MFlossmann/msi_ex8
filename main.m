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

robot_y = figure;
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

figure(robot_y);
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

figure(robot_y);
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

figure(robot_y);
hold on;
plot(1:size(rls_fit, 1),rls_fit(:,2),'--', 'DisplayName', 'RLS smooth')
legend('show');
hold off;