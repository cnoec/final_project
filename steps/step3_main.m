%% Initialization

T_end                               =       26;
n_iterations                        =       T_end/Ts;

load('step2.mat','u_opt_step2');

u_0                                 =       u_opt_step2;

clear u_opt_step2

u_0_2                               =       zeros(n_iterations+3,1);
u_0_2(1:end-1)                      =       u_0(1:length(u_0_2)-1);
u_0_2(end)                          =       0.1;
 
u_0_step3                           =       u_0_2;

clear u_0_2

%% Constrained algorithm settings

myoptions                           =       con_optimalset;
myoptions.gradmethod                =       'CD';
myoptions.graddx                    =       2^-17;
myoptions.tolgrad                   =       1e-8;
myoptions.tolfun                    =       1e-17;
myoptions.tolx                      =       1e-16;
myoptions.ls_beta                   =       0.8;
myoptions.ls_c                      =       .1;
myoptions.ls_nitermax               =       1e2*3;
myoptions.nitermax                  =       50;
myoptions.xsequence                 =       'on';   

%% Constrained optimization

p                                   =       0;                  %# of nonlinear equality constraints
q                                   =       5+n_iterations;     %# of nonlinear inequality constraints

str                                 =       "yes";

tic

[u_opt_step3,~,~,~,seq_step3]       =       con_NLP_opt(@(u)( fun(u,xi0, T_end, Ts, ...
                                            waypoints, n_wp , innerBoundary, outerBoundary,6,str)),...
                                            u_0_step3,[],[],[],[],p,q,myoptions,"step3_iterations");

con_time                            =       toc;

%% Trajectory generation

[xi_step3, t_vec, ~,torque]         =       trajectory_generation_cc(u_opt_step3, xi0, T_end, Ts,1e-2,"");
[xi_in_step3, t_vec, ~,torque]      =       trajectory_generation_cc(u_0_step3, xi0, T_end, Ts,1e-2,"");

save('mat_data\step3.mat','xi_step3','u_opt_step3','seq_step3','t_vec','torque');

%%