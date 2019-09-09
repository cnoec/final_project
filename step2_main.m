%% Initialization

load('step1.mat','u_opt_step1');

% u_0_step2 = zeros(T_end/Ts_steer + 2, 1);
u_0_step2 = [20; 15; u_opt_step1; ones(272-252, 1)];
u_0_step2(58:end) = 0.6/15;

%% times

T_end_step2 = 27;

%% Unconstrained optimization

tic
[u_opt_step2,~,debug_step2]       =       unc_NLP_opt(@(u)(cost_function_gamma(u,xi0,T_end_step2,Ts,...
                                                                    waypoints,n_wp,0,1e6)),u_0_step2,unc_optimalset,"step2_iterations");
step2_time                  =       toc;
                           
%% Trajectory generation 

[xi_in_step2, ~, ~, ~]              =       trajectory_generation_cc(u_0_step2, xi0, T_end_step2, Ts, Ts_sim);
[xi_step2, ~, ~, ~]                 =       trajectory_generation_cc(u_opt_step2, xi0, T_end_step2, Ts,Ts_sim);

save('mat_data\step2.mat','xi_step2','u_opt_step2','debug_step2');

%%