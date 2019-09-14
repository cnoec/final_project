function [dist_min] = wp_to_trajectory_distance(target, traject, n_wp,n_states)
% wp_to_trajectory_distance computes the minimum distance between the
% target points and the trajectory. It is also possible to set a string cmd
% in order to compute the minimum distance from all the target points.
% INPUTS
%       target:      vector conteining the target points
%       trajectory:  matrix conteining the trajectory, on the first column
%                    we'll find the x coordinate, while on the second one 
%                    we'll have the y ones.
%       cmd:         string which allows the user to use this function to
%                    compute:
%                               -'all' : the function will return a vector
%                                        composed by the minimum distances
%                                        from all the target points.
%                               -'only': the function will return a scalar
%                                        which represents the minimum
%                                        distance between the input target
%                                        point and the input trajectory.
%
% OUTPUTS
%       ditance:    minimum distance

% Initialization in order to let work the algorithm
traject     =       traject';
max_dist    =       10e20;

min_index = 1;

dist_min = zeros(n_wp,1);
dist_min(1:n_wp,:) = max_dist;
weight = zeros(n_states,1);

for i = 1:n_wp
    for j = 1:n_states
         dist = norm(target(i,1:2) - traject(j,1:2)) + weight(j);
         if ( dist < dist_min(i) )
            dist_min(i) = dist;
            min_index = j;
         end
    end
%     weight(1:min_index) = max_dist;
%     plot(traject(min_index,1),traject(min_index,2),'.b')
%     txt = {i};
%     text(traject(min_index,1)+1,traject(min_index,2)+1,txt);
end
  

end