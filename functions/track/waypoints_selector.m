function [ waypoints ] = waypoints_selector(innerBoundary,outerBoundary, n_wp,N)
% descrizione da scrivere una volta aggiunti anche i cerchi e i raggi 
%
% Inputs:   innerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the outer boundary
%                                   samples. 
%           N                   -   number of boundary samples
%           n_wp                -   number of waypoints decided by the user
%                                   in the main file (start and finish included)
%
% Outputs:  waypoints           -   matrix n_wpx3 - it contains the x,y,
%                                   coordinates of the waypoints, decided
%                                   as the middle point of the track.

waypoints = zeros(n_wp,3);

count = n_wp;

for i=1:count
    x   = [ innerBoundary(floor(i*N/count),1,1)  outerBoundary(floor(i*N/count),1,1)]';
    y   = [ innerBoundary(floor(i*N/count),2,1)  outerBoundary(floor(i*N/count),2,1)]';
    waypoints(i,1,1) =  mean(x);
    waypoints(i,2,1) =  mean(y);
    plot(waypoints(i,1,1),waypoints(i,2,1),'.g')
    txt = {i};
    text(waypoints(i,1,1)+1,waypoints(i,2,1)+1,txt);
end
