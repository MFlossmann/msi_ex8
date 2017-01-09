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

theta= pinv(Phi)*XYm;

%fit = @(k,theta) theta'*[k.^4;k.^3;k.^2;k;ones(1,size(k,2))];
%lls_fit = fit(linspace(1,80),theta)';

lls_fit= Phi*theta;

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
plot(t,lls_fit(:,1),'-', 'DisplayName', 'LLS fit')
%legend('show');
title('x position and estimate')
hold off;

rob_y = figure;
hold on;
plot(t,XYm(:,2),'o', 'DisplayName', 'True positions')
plot(t,lls_fit(:,2),'-', 'DisplayName', 'LLS fit')
%legend('show');
title('y position and estimate')
hold off;

%% 1.b)

theta_RLS = ones(d,control_dim);
Q = eye(d);
rls_fit = zeros(N,control_dim);

for i = 1:N
    [theta_RLS, Q] = RLS(theta_RLS, Q, Phi(i,:)', XYm(i,:), 1);
    rls_fit(i,:)= Phi(i,:)*theta_RLS;
end

figure(robot_2D);
hold on;
plot(rls_fit(:,1),rls_fit(:,2), '-', 'DisplayName', 'RLS fit')
%legend('show');
hold off;

figure(rob_x);
hold on;
plot(t,rls_fit(:,1),'-', 'DisplayName', 'RLS fit')
%legend('show');
hold off;

figure(rob_y);
hold on;
plot(t,rls_fit(:,2),'-', 'DisplayName', 'RLS fit')
%legend('show');
hold off;

%% 1.c)

% jagged?

theta_RLS_jagged = zeros(d,control_dim);
Q_jagged = eye(d);
rls_fit_jagged = zeros(N,control_dim);
for i = 1:N
    [theta_RLS_jagged, Q_jagged] = RLS(theta_RLS_jagged, Q_jagged, Phi(i,:,:)', XYm(i,:), 0.5);
    rls_fit_jagged(i,:) = Phi(i,:)*theta_RLS_jagged;
end

figure(robot_2D);
hold on;
plot(rls_fit_jagged(:,1),rls_fit_jagged(:,2), '--', 'DisplayName', 'RLS jagged')
%legend('show');
hold off;

figure(rob_x);
hold on;
plot(t,rls_fit_jagged(:,1),'--', 'DisplayName', 'RLS jagged')
%legend('show');
hold off;

figure(rob_y);
hold on;
plot(t,rls_fit_jagged(:,2),'--', 'DisplayName', 'RLS jagged')
%legend('show');
hold off;

%% smooth?
theta_RLS_smooth = zeros(d,control_dim);
Q_smooth = eye(d);
rls_fit_smooth = zeros(N,control_dim);
for i = 1:N
    [theta_RLS_smooth, Q_smooth] = RLS(theta_RLS_smooth, Q_smooth, Phi(i,:,:)', XYm(i,:), 0.85);
    rls_fit_smooth(i,:) = Phi(i,:)*theta_RLS_smooth;
end

figure(robot_2D);
hold on;
plot(rls_fit_smooth(:,1),rls_fit_smooth(:,2), '--', 'DisplayName', 'RLS smooth')
legend('show');
hold off;

figure(rob_x);
hold on;
plot(t,rls_fit_smooth(:,1),'--', 'DisplayName', 'RLS smooth')
legend('show');
hold off;

figure(rob_y);
hold on;
plot(t,rls_fit_smooth(:,2),'--', 'DisplayName', 'RLS smooth')
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

numPoints= 50;
% generate the coordinates of |numPoints| points on a unit circle
xy_circle=[cos(linspace(0,2*pi,numPoints)); sin(linspace(0,2*pi,numPoints))];
%generate raw 3D-ellipse data matrix to store all |N| ellipsoids for
%increasing amount of data points
ellipse= zeros(control_dim, numPoints, N);

for i = 1:N
    
    if i < N
        [theta_RLS, Q] = RLS(theta_RLS, Q, Phi(i+1,:)', XYm(i+1,:), 0.85);
    
        % extrapolate
        osa_fit(i,:) = Phi(i,:)*theta_RLS;
        
    
        % calculate sigmas for theta and one step ahead estimates
%         for j = 1:control_dim
%             sigma_theta(:,:,j) =(norm(XYm(1:i,j) - Phi(1:i,:) * theta_RLS(:,j))/(N-d))* ...
%                                     inv(Phi(1:i,:)'*Phi(1:i,:));
%             %warum ist z.B. erste Zeile inv?
%             sigma_osa(i,j) = Phi(i+1,:)*sigma_theta(:,:,1)*Phi(i+1,:)';
%             
%         end
        %compute covariance matrix for data points
        cov_osa= cov(osa_fit(1:i,:));
        % Compute the eigenvalues and eigenvectors
        % of the covariance matrice
        [V1,D1] = eig(cov_osa);
        % generate the points of the confidence ellipse
        ellipse(:,:,i) = [osa_fit(i,1); osa_fit(i,2)]*ones(1,numPoints) + V1*sqrt(D1)*xy_circle;

    end
end

% Redundant...berechne letzte Reihe
osa_fit(i,:) = Phi(i,:)*theta_RLS;

cov_osa= cov(osa_fit(1:i,:));
% Compute the eigenvalues and eigenvectors
% of the covariance matrices
[V1,D1] = eig(cov_osa);
% generate the points of the confidence ellipse
ellipse(:,:,i) = [osa_fit(i,1); osa_fit(i,2)]*ones(1,numPoints) + V1*sqrt(D1)*xy_circle;

figure(robot_2D);
hold on;
plot(osa_fit(:,1),osa_fit(:,2), 'r-', 'DisplayName', 'OSA extrap.')
for i= 1:size(ellipse,3)
    % plot the ellipses
    %plot(ellipse(1,:,i), ellipse(2,:,i), 'k', 'DisplayName', 'Conf. ellipsoids');
    %Ellipsoids get bigger...
    %pause(0.25);
end

%kein Effekt?
legend('show');
hold off;

figure(rob_x);
hold on;
plot(t,osa_fit(:,1),'r-', 'DisplayName', 'OSA extrap.')
legend('show');
hold off;

figure(rob_y);
hold on;
plot(t,osa_fit(:,2),'r-', 'DisplayName', 'OSA extrap.')
legend('show');
hold off;

%% 2.a) b) c)
%Script page 52:
% "Function f is is a map from states, controls and time to the rate of
% change of the state"
R_L= 0.2; %[m]
R_R= 0.2; %[m]
L= 0.6; %[m]
param = [R_L; R_R; L];
u= u*360;
%init values
x0 = [0;0;0];
% 
% xdot= robot_ode(0, 0, u(1,:), param);
% 
% x_next= euler_step(1, x0, u, @robot_ode, param);

t_sample= 0.0159;
%time vector for robot's path
t= 0:t_sample:t_sample*(size(u,1)-1);

%simulate robot's path using euler steps
x_sim_euler= sim_euler(t, x0, u, param);

%System output [x y]'
y_euler= x_sim_euler(1:2,:);

robot_sim= figure;
plot(y_euler(1,:), y_euler(2,:), 'r', 'DisplayName', 'Euler method');
title('2D robot path simulation')
hold on;

%% 2.d)
%Runge-Kutta integrator of order 4

%simulate robot's path using RK4
x_sim_rk= sim_rk4(t, x0, u, param);

%System output [x y]'
y_rk= x_sim_rk(1:2,:);

figure(robot_sim)
plot(y_rk(1,:), y_rk(2,:), 'b', 'DisplayName', 'Runge-Kutta method of order 4');

legend('show');

% figure(5)
% plot(t, u(:,1), 'r', 'DisplayName', 'left wheel');
% hold on;
% plot(t, u(:,2), 'b', 'DisplayName', 'right wheel');
% legend('show')





