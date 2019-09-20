function v = fun(u, xi0, T_end, Ts, waypoints, n_wp ,inner_boundary,outer_boundary,road_width,str)
% Cost function that is used in step 3 for constrained optimization. it
% considers the constraints and their tuning. 
%  
%   INPUTS:
%              u                =   input
%              xi0              =   initial conditions  
%              T_end            =   simulation time
%              Ts               =   time step for the steering sampling
%              waypoints        =   waypoint matrixes
%              n_wp             =   number of waypoints decided by the user
%                                   in the main file
%              innerBoundary    =   matrix Nx2 - it contains the x,y
%                                   coordinates of the inner boundary
%                                   samples. 
%              outerBoundary    =   matrix Nx2 - it contains the x,y
%                                   coordinates of the outer boundary
%                                   samples. 
%              road_width       =   road width in meters
%
%   OUTPUTS:
%              v                =   vector containing the tuned cost
%                                   function, the tuned equality
%                                   constraints and the tuned inequality
%                                   constraints

% Initialization
Ts_sim      =       1e-2;
n_states    =       T_end/Ts_sim;

%% Cost function

[F,~,xi]    =       cost_function_gamma(u,xi0,T_end,Ts,waypoints,n_wp,7,1,str);

gamma       =       u(end);

F           =       (F + gamma*1e3)/1e6;

%% Equality constraints

g           =       [];

limit       =       ones(length(xi(3,:)),1)*22;


%% Inequality Constraints

h_soft      =       [-(xi(2,end)-xi(2,1))+road_width/2;
                      (xi(2,end)-xi(2,1))+road_width/2;
                     -(xi(1,end)-xi(1,1))+gamma;
                      (xi(1,end)-xi(1,1))+gamma;];

h           =       [h_soft; 
                    track_constraint(xi,inner_boundary,outer_boundary,n_states,road_width)];

h           =       h/1e4;

%% Output

v           =       [F;
                     g;
                     h];

end