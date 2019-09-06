function [track,innerBoundary,outerBoundary,N,x0,y0] = track_generation(track_number)

% auxiliary function that generates the track and sets the initial
% position. change the function called by track to change the shape.
% 
% Inputs:   track_number        -   1 ->  CIRCLE TRACK
%                                   2 ->  OVAL TRACK
%                                   3 ->  RANDOM TRACK
%
% Outputs:  track               -   matrix  
%           innerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the outer boundary
%                                   samples. 
%           N                   -   number of boundary samples
%           x0                  -   initial x coordinate
%           y0                  -   initial y coordinate


% create and plot track

if track_number == 1
    track = circle_track();
elseif track_number == 2
    track = oval_track();
elseif track_number == 3
    track = random_track();
end

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

% plot of track
%plot(track)
%camroll(-90)
% figure
% plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
%     outerBoundary(:,2),'black'),grid on
% axis equal
% hold on

% plot of finish line
% line([innerBoundary(1,1,1) outerBoundary(1,1,1)],...
%      [innerBoundary(1,2,1) outerBoundary(1,2,1)],'color','y','linewidth', 7) 
% axis equal
% hold on

x0  = mean([innerBoundary(1,1,1) outerBoundary(1,1,1)]');
y0  = mean([innerBoundary(1,2,1) outerBoundary(1,2,1)]');

innerBoundary(:,3) = [];
outerBoundary(:,3) = [];

end
 