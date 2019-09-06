%% Track setting

% choose which track to plot:
% track_number = 1 -> CIRCLE TRACK
% track_number = 2 -> OVAL TRACK
% track_number = 3 -> RANDOM TRACK

track_number        =   2;
[~,outerBoundary,innerBoundary,N,x0,y0] = track_generation(track_number);

n_wp                =   30;
[ waypoints ] = waypoints_selector(innerBoundary,outerBoundary, n_wp,N);

%% Initial Guess Setting

m                   =   (outerBoundary(1,2)-innerBoundary(1,2))/(outerBoundary(1,1)-innerBoundary(1,1));
m                   =   -1/m;

X                   =   x0;         % inertial X position (m)
Y                   =   y0;         % inertial Y position (m)
Ux                  =   20;         % body x velocity (m/s)
beta                =   0;          % sideslip angle (rad)
psi                 =   atan(m);    % yaw angle (rad)

if (track_number == 1) 
    
    psi             =   psi + pi; 

end

r                   =       0;          % yaw rate (rad/s)
xi0                 =       [X Y Ux beta psi r]';

T_end               =       26;
Ts                  =       1e-1;
Ts_sim              =       1e-2;
n_iterations        =       T_end/Ts;
% n_states            =       T_end/Ts_sim;

%% cruise control
global num_dyn den_dyn num_int

s = tf('s');

CC_int              =   90/s;
CC_int_d            =   c2d(CC_int, Ts_sim, 'tustin');
[num_int, den_int]  =   tfdata(CC_int_d, 'v');

CC_dyn              =   920/(s/10 + 1);
CC_dyn_d            =   c2d(CC_dyn, Ts_sim, 'tustin');
[num_dyn, den_dyn]  =   tfdata(CC_dyn_d, 'v');

%% h_track variables

global Par_1 Par_2 Par_3 Par_4 Par_5 Par_6 Par_7 Par_8


Par_3               =   CircleFitByTaubin(outerBoundary(30:98,:));
Par_4               =   CircleFitByTaubin(innerBoundary(30:98,:));

Par_7               =   CircleFitByTaubin(outerBoundary(158:228,:));
Par_8               =   CircleFitByTaubin(innerBoundary(158:228,:));

Par_1               =   -Par_3(3)+Par_4(2);
Par_2               =   -Par_4(3)+Par_4(2);

Par_5               =   Par_3(3)+Par_4(2);
Par_6               =   Par_4(3)+Par_4(2);

%%