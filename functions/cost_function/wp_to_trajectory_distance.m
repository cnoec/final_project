function [dist_min] = wp_to_trajectory_distance(target, traject, n_wp,n_states)
% wp_to_trajectory_distance computes the minimum distance between the
% target points and the trajectory.
%
% INPUTS
%       target      =   target points vector
%       trajectory  =   x-y coordinates of the different states of the
%                       vehicle
%       n_wp        =   number of waypoints
%       n_states    =   trajectory length 
%
% OUTPUTS
%       ditance     =   minimum distance from the trajectory to each
%                       waypoint

% Initial settings
traject     =       traject';
max_dist    =       10e20;

% Initialization of the output vector that becomes a vector with very big
% elements. Whenever a smaller value is found, the output vector is
% updated.
dist_min = zeros(n_wp,1);
dist_min(1:n_wp,:) = max_dist;
weight = zeros(n_states,1);

% Minimimum distance computation
for i = 1:n_wp
    for j = 1:n_states
         dist = norm(target(i,1:2) - traject(j,1:2)) + weight(j);
         if ( dist < dist_min(i) )
            dist_min(i) = dist;
         end
    end
end
  

end