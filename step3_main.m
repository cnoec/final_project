%% Initialization
u_d                                 =       ones(n_iterations,1)*3*pi/180;

T_end                               =       26;
n_iterations                        =       T_end/Ts;

load('step2.mat','u_opt_step2');

u_0                                 =       u_opt_step2;

clear u_opt_step2

u_0_2                               =       zeros(n_iterations+2,1);
u_0_2(1:length(u_0))                =       u_0(1:end);
u_0_2(length(u_0)+1:end)            =       u_0(end);
 
u_0                                 =       u_0_2;

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

p                                   =       1;                  %# of nonlinear equality constraints
q                                   =       3+n_iterations;     %# of nonlinear inequality constraints

tic

[u_opt_step3,~,~,~,seq_step3]       =       con_NLP_opt(@(u)( fun(u,xi0, T_end, Ts, ...
                                            waypoints, n_wp , innerBoundary, outerBoundary,6)),...
                                            u_0,[],[],[],[],p,q,myoptions,"step3_iterations");

con_time                            =       toc;

%% Trajectory generation

[xi_step3, t_vec, ~,torque]         =       trajectory_generation_cc(u_opt_step3, xi0, T_end, Ts,1e-2);

save('mat_data\step3.mat','xi_step3','u_opt_step3','seq_step3','t_vec','torque');

%%