% Constrained Numerical Optimization for Estimation and Control
% OPTIMIZATION PROJECT 

clear all
close all
clc

path        =           pwd;
addpath('functions');
addpath('model');

%% track generation and waypoints positioning

track_number = 2;
[~,outerBoundary,innerBoundary,N,x0,y0] = track_generation(track_number);

n_wp = 30;
[ waypoints ] = waypoints_selector(innerBoundary,outerBoundary, n_wp,N);

%% parameters initialization and setting of initial state

m = (outerBoundary(1,2)-innerBoundary(1,2))/(outerBoundary(1,1)-innerBoundary(1,1));
m = -1/m;

X       =       x0;         % inertial X position (m)
Y       =       y0;         % inertial Y position (m)
Ux      =       20;         % body x velocity (m/s)
beta    =       0;          % sideslip angle (rad)
psi     =       atan(m);    % yaw angle (rad)
r       =       0;          % yaw rate (rad/s)
xi0     =       [X Y Ux beta psi r]';

plot(X,Y,'*r');

%% simulation

boundary_number     =       1;
tau                 =       0;
d                   =       0;
Ts                  =       1e-1; 
T_end               =       25;
n_iterations        =       T_end/Ts;

% u                                   =       ones(2*n_iterations,1);
% u(1:n_iterations)                   =       100;
% u(n_iterations+1:2*n_iterations)    =       0*pi/180;

u_d                                 =       ones(n_iterations,1)*3*pi/180;
u_T                                 =       ones(n_iterations,1)*100;
[xi, t_vec, end_check]              =       trajectory_generation([u_T;u_d], xi0, T_end, Ts);

n_states                            =       length(xi);

tic
[u_opt,exit_flag,seq]       = uncons_NLP_opt(@(u_opt)(deltasum(u_opt, u_T ,xi0, T_end, Ts, waypoints, n_wp)...
                                    ),u_d,unc_optimalset);
toc

%%        

[xi_ini, ~, ~]    = trajectory_generation([u_T;u_d], xi0, T_end, Ts);
[xi, ~, ~]    = trajectory_generation([u_T;u_opt], xi0, T_end, Ts);

figure('Name', 'Optimal Trajectory' , 'NumberTitle', 'off')
subplot 211
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on, axis equal
hold on
plot(xi_ini(1,:), xi_ini(2,:), '.blue');
hold off

subplot 212
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on, axis equal
hold on
plot(xi(1,:), xi(2,:), '.r');
hold off

% for i=1:(n_states-1)
%    plot([xi(1,i) xi(1,i+1)],[xi(2,i) xi(2,i+1)],'.r');
% end

% figure
% 
% for i = 1:min(size(seq))
%     figure
%     plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
%         outerBoundary(:,2),'black'),grid on
%     axis equal
%     hold on
%     u_check                     =       seq(:,i);
%     u = [u_T; u_check];
%     [xi_1, ~, ~]                =       trajectory_generation(u, xi0, T_end, Ts);
%     plot(xi_1(1,:), xi_1(2,:),'.');grid;
%     title(i);
%     pause(0.5)
%     close
% end

%
% dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

% dist = zeros(n_wp,1);
% min_dist_point = zeros(n_wp,2);
% min_index = 1;
 
% F   = fopen('prova.txt','w');
% fprintf(F,'Gradient \n');
% fprintf(F,'%f           %f \n',y,grad);

%%
