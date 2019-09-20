function [track,innerBoundary,outerBoundary,N,x0,y0] = track_generation()
% function that generates the track and sets the initial position.
%
% OUTPUTS:  
%           track               =   object with all the track information
%           innerBoundary       =   matrix Nx2 - it contains the x,y
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       =   matrix Nx2 - it contains the x,y
%                                   coordinates of the outer boundary
%                                   samples. 
%           N                   =   number of boundary samples
%           x0                  =   initial x coordinate
%           y0                  =   initial y coordinate


% create and plot track
track = oval_track();

% find inner and outer boundaries coordinates
rb = roadBoundaries(track);
outerBoundary = rb{2};
innerBoundary = rb{1};
        
N = length(innerBoundary);

% remapping of inner and outer boundaries matrixes
aux1 = zeros(N,3);
aux2 = zeros(N,3);

for i=1:N
   if(i==N)
       aux1(i,:) = outerBoundary(1,:); 
       aux2(i,:) = innerBoundary(2,:);
   else
       aux1(i,:) = outerBoundary(N-i+1,:); 
       aux2(i,:) = innerBoundary(i+1,:);
   end
end

innerBoundary = aux2;
outerBoundary = aux1;
clear aux1 aux2 

innerBoundary(:,3) = [];
outerBoundary(:,3) = [];

% initial position
x0  = mean([innerBoundary(1,1,1) outerBoundary(1,1,1)]');
y0  = mean([innerBoundary(1,2,1) outerBoundary(1,2,1)]');



end
 