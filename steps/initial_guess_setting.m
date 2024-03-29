% Script for initial track settings and initial state conditions. Cruise
% control equations and timings definition. 

%% Track generation
[~,outerBoundary,innerBoundary,N,x0,y0] =   track_generation();

% number of waypoints decided by the user
n_wp                                    =   30;

[ waypoints ]                           =   waypoints_selector(innerBoundary,outerBoundary, n_wp,N);

%% Initial state setting
m                   =   (outerBoundary(1,2)-innerBoundary(1,2))/(outerBoundary(1,1)-innerBoundary(1,1));
m                   =   -1/m;
Ux                  =   20;                         % body x velocity (m/s)
beta                =   0;                          % sideslip angle (rad)
psi                 =   atan(m);                    % yaw angle (rad)
r                   =   0;                          % yaw rate (rad/s)
xi0                 =   [x0 y0 Ux beta psi r]';

%% Time
T_end               =       25;
Ts                  =       1e-1;
Ts_sim              =       1e-2;
n_iterations        =       T_end/Ts;

%% cruise control
global num_dyn den_dyn num_int

s = tf('s');

CC_int              =   90/s;
CC_int_d            =   c2d(CC_int, Ts_sim, 'tustin');
[num_int, den_int]  =   tfdata(CC_int_d, 'v');

CC_dyn              =   920/(s/10 + 1);
CC_dyn_d            =   c2d(CC_dyn, Ts_sim, 'tustin');
[num_dyn, den_dyn]  =   tfdata(CC_dyn_d, 'v');
