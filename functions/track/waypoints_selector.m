function [ waypoints ] = waypoints_selector(innerBoundary,outerBoundary, n_wp,N)
% function that chooses a priori the position of the waypoints, fiven the
% number desired by the user.
%
% INPUTS:
%           innerBoundary       -   matrix Nx2 - it contains the x,y
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       -   matrix Nx2 - it contains the x,y
%                                   coordinates of the outer boundary
%                                   samples. 
%           N                   -   number of boundary samples
%           n_wp                -   number of waypoints decided by the user
%                                   in the main file
%
% OUTPUTS:  
%           waypoints           -   matrix n_wpx2 - it contains the x,y
%                                   coordinates of the waypoints, decided
%                                   as the middle point of the track.

% initialization
waypoints = zeros(n_wp,3);
count = n_wp;

% computation of waypoints
for i=1:count
    x   = [ innerBoundary(floor(i*N/count),1,1)  outerBoundary(floor(i*N/count),1,1)]';
    y   = [ innerBoundary(floor(i*N/count),2,1)  outerBoundary(floor(i*N/count),2,1)]';
    waypoints(i,1,1) =  mean(x);
    waypoints(i,2,1) =  mean(y);
end
