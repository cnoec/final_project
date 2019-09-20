%% Step 2: Unconstrained Mixed approach
% a feedback cruise control is adopted. The torque is no longer an 
% optimization variable, but the reference speeds are. 
% 
% cost function used: cost_function_gamma
% simulation time length: 27s
% cruise control: yes
% input: two speed references + output of step 1

%% Initialization
load('step1.mat','u_opt_step1');

u_0_step2                           =       [   20; 
                                                15; 
                                                u_opt_step1; 
                                                ones(272-252, 1)];
                                            
u_0_step2(58:end)                   =       0.6/15;

%% time
T_end_step2                         =       27;

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
[u_opt_step2,~,debug_step2]         =       unc_NLP_opt(@(u)(cost_function_gamma(u,xi0,T_end_step2,Ts,...
                                                             waypoints,n_wp,0,1e6,"")),u_0_step2,myoptions,"step2_iterations");
step2_time                          =       toc;
                           
%% Trajectory generation 
[xi_in_step2, ~, ~, ~]              =       trajectory_generation_cc(u_0_step2, xi0, T_end_step2, Ts, Ts_sim,"");
[xi_step2, ~, ~, ~]                 =       trajectory_generation_cc(u_opt_step2, xi0, T_end_step2, Ts,Ts_sim,"");

save('mat_data\step2.mat','xi_step2','u_opt_step2','debug_step2');

%%