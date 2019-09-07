%% Initialization

load('step1.mat','u_opt_step1');

u_opt_step1(68:end)                 =       0.5/15;
u_0                                 =       [20; 
                                             15; 
                                             u_opt_step1];

%% Unconstrained optimization

tic
[u_opt_step2,~,~]                   =       unc_NLP_opt(@(u)(cost_function_gamma(u,xi0,T_end,Ts,...
                                            waypoints,n_wp,20,1e6)),u_0,unc_optimalset,"step2_iterations");
step2_time                          =       toc;
                           
%% Trajectory generation 

[xi_step2, ~, ~, ~]                 =       trajectory_generation_cc(u_opt_step1, xi0, T_end, Ts,Ts_sim);

save('mat_data\step2.mat','xi_step2','u_opt_step2');

%%