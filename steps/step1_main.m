%% Step 1: Unconstrained Open Loop approach
% the goal is to minimize the sum of the squared distances of the overall
% trajectory from the waypoints of the track
% 
% cost function used: deltasum
% simulation time length: 27s
% cruise control: no
% input: constant torques and constant steering angles

%% Initialization
u_d                                 =       ones(n_iterations,1)*3*pi/180;
u_T                                 =       ones(n_iterations,1)*100;

%% Constrained algorithm settings
myoptions                           =       unc_optimalset;
myoptions.gradmethod                =       'CD';
myoptions.graddx                    =       2^-17;
myoptions.tolgrad                   =       1e-6;
myoptions.tolfun                    =       1e-12;
myoptions.tolx                      =       1e-12;
myoptions.ls_beta                   =       0.8;
myoptions.ls_c                      =       .1;
myoptions.ls_nitermax               =       300;
myoptions.nitermax                  =       1000;
myoptions.xsequence                 =       'on';   

%% Unconstrained optimization
tic
[u_opt_step1,exit_flag,seq]         =       unc_NLP_opt(@(u_opt)(deltasum(u_opt, u_T ,xi0, T_end, Ts,...
                                            waypoints, n_wp)),u_d,myoptions,"step1_iterations");
step1_time                          =       toc;

%% Trajectory generation       
[xi_in_step1, t_vec, end_check]     =       trajectory_generation([u_T;u_d], xi0, T_end, Ts);
[xi_step1, ~, ~]                    =       trajectory_generation([u_T;u_opt_step1], xi0, T_end, Ts);

save('mat_data\step1.mat','xi_in_step1','xi_step1','u_opt_step1');

%%
