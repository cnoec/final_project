clear all
close all
clc

path                    =   pwd;
addpath('Functions');
addpath('Model');
addpath('Mat_Data\');
addpath('Functions\unc_optimization');
addpath('Functions\con_optimization');
addpath('Functions\track');

addpath('Functions\cost_function');

%% Run the initialization

%run('vehicle_project_UNCONSTRAINED.m');
load('310819_uopt.mat');

u_0                     =   u_opt;
clear u_opt

run('initial_guess_setting');

%% Constrained algorithm settings

myoptions               =   con_optimalset;
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.tolfun        =   1e-17;
myoptions.tolx          =	1e-16;
myoptions.ls_beta       =	0.8;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2*3;
myoptions.nitermax      =	50;
myoptions.xsequence     =	'on';   

% (fun,x0,A,b,C,d,p,q,con_options,filename)

% u_0                     =   [20;
%                              25;
%                              0.1;
%                              3/15*ones(T_end/Ts,1)];

% u_0                       =    [u_0(1:2);
%                                 0.1;
%                                 u_0(3:end);];

u_0_2                   =       zeros(n_iterations+2,1);
u_0_2(1:length(u_0))    =       u_0(1:end);
u_0_2(length(u_0)+1:end)=       u_0(end);


% u_0_2                   =   zeros(n_iterations+3,1);
% u_0_2(1:length(u_0))    =   u_0;
% u_0_2(length(u_0)+1:end)=   u_0(end);
% 
u_0                     =   u_0_2;

clear u_0_2
                         
%% Initial Setting visualization

[xi_0,~,~,~]              =    trajectory_generation_cc(u_0, xi0, T_end, Ts,1e-2);


figure('Name','Initial_Guess')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
     outerBoundary(:,2),'black');grid on;xlabel('m');ylabel('m');
axis equal
hold on
plot(xi_0(1,:),xi_0(2,:))

%% optimization

p                       =   1;          %# of nonlinear equality constraints
q                       =   3+2600/10;  %# of nonlinear inequality constraints
% q                       =   2;

tic

C = zeros(1,length(u_0));

C(3) = 1;

D = 0;

[u_opt,~,~,exit,seq]    =   con_NLP_opt(@(u)( fun(u,xi0, T_end, Ts, ...
                            waypoints, n_wp , innerBoundary, outerBoundary,6)),...
                            u_0,[],[],C,D,p,q,myoptions,"con_iterations");

con_time                =   toc;

%% Optimal trajectory plot

[xi, t_vec, ~,torque]   =   trajectory_generation_cc(u_opt, xi0, T_end, Ts,1e-2);

figure('Name','Optimal_Solution')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
     outerBoundary(:,2),'black');grid on;xlabel('m');ylabel('m');
axis equal
hold on
plot(xi(1,:),xi(2,:))

figure('Name','Debug_torque')
plot(t_vec(1:end-1),torque);grid;title('Torque');xlabel('time [s]');ylabel('Torque [Nm]');

figure('Name','Debug_speed')
plot(t_vec,xi(3,:));grid;title('Speed');xlabel('time [s]');ylabel('Speed [m/s]');

%% sequence

figure('Name','Complete_sequence')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
     outerBoundary(:,2),'black');grid on;xlabel('m');ylabel('m');
axis equal
hold on

for j = 1:size(seq,2)
    
    [xi_s,~,~,~]        =    trajectory_generation_cc(seq(:,j), xi0, T_end, 0.1,1e-2);
    plot(xi_s(1,:),xi_s(2,:));
    hold on
    
end

% i                       =    84;
% 
% plot_ith_sequence(seq,i,innerBoundary,outerBoundary,xi0,T_end)

%%