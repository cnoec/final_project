function [sum_Delta] = deltasum(u_d,u_T, xi0, T_end, Ts, waypoints, n_wp)
% Cost function that is used in step 1, in which the optimization variables
% are both the torques and the steering angles.
%
%   INPUTS:
%           u_d          =   vector of steering angles
%           u_T          =   vector of torque inputs
%           xi0          =   initial guess for the optimization variables
%           T_end        =   seconds available to complete one lap.
%           Ts           =   sampling time
%           waypoints    =   waypoint matrixes
%           n_wp         =   number of waypoints
%
%   OUTPUTS:
%           sum_Delta    =   sum of the squared distances of the overall 
%                            trajectory from the waypoints of the track

% initialization 
u = [u_T;
     u_d;];

% trajectory generation
[xi, ~, ~] = trajectory_generation(u, xi0, T_end, Ts);

n_states = length(xi);

% distance computation
dist = wp_to_trajectory_distance( waypoints, xi(1:2,:),n_wp,n_states);

% cost function 
sum_Delta = dist'*dist/1e7;

end

