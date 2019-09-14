%% Initialization

u_d                                 =       ones(n_iterations,1)*3*pi/180;
u_T                                 =       ones(n_iterations,1)*100;

%% Unconstrained optimization

tic
[u_opt_step1,exit_flag,seq]         =       unc_NLP_opt(@(u_opt)(deltasum(u_opt, u_T ,xi0, T_end, Ts,...
                                            waypoints, n_wp)),u_d,unc_optimalset,"step1_iterations");
step1_time                          =       toc;

%% Trajectory generation       

[xi_in_step1, t_vec, end_check]     =       trajectory_generation([u_T;u_d], xi0, T_end, Ts);
[xi_step1, ~, ~]                    =       trajectory_generation([u_T;u_opt_step1], xi0, T_end, Ts);

save('mat_data\step1.mat','xi_in_step1','xi_step1','u_opt_step1');

%%
