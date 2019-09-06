function in_track = track_constraint(xi,innerBoundary,outerBoundary,n_states,road_width)
%
% Inputs:   xi                  -   state vector returned by the simulation
%           innerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the inner boundary
%                                   samples. 
%           outerBoundary       -   matrix Nx3 - it contains the x,y,z
%                                   coordinates of the outer boundary
%                                   samples. 
%           n_states            -   number of states the simulation returns
%           road_width          -   road track width decided in the main file
%
% Outputs:  in_track            -   if the i-th number in the in_track vector is >0,
%                                   the car is in the track
%                                   if the i-th number in the in_track vector is <0, 
%                                   the car is out of the track

dist        =   zeros(n_states/10,2);
in_track    =   zeros(n_states/10,1);

j = 1;

for i = 1:10:n_states 
    
   inBound_k            =  [ innerBoundary(j,1,1)  innerBoundary(j,2,1)]; 
   outBound_k           =  [ outerBoundary(j,1,1)  outerBoundary(j,2,1)];
   
   if( j ~= length(innerBoundary) )
       inBound_kp1      =  [ innerBoundary(j+1,1,1)  innerBoundary(j+1,2,1)]; 
       outBound_kp1     =  [ outerBoundary(j+1,1,1)  outerBoundary(j+1,2,1)];
   else
       inBound_kp1      =  [ innerBoundary(1,1,1)  innerBoundary(1,2,1)]; 
       outBound_kp1     =  [ outerBoundary(1,1,1)  outerBoundary(1,2,1)];
   end
   
   dist(j,1)            =   point_to_line_distance(xi(1:2,i)',inBound_k,inBound_kp1);
   dist(j,2)            =   point_to_line_distance(xi(1:2,i)',outBound_k,outBound_kp1);
    
   in_track(j)          =   road_width - (dist(j,1) + dist(j,2));

   j = j+1;
   
   if (j == length(innerBoundary))
       
       return;
       
   end
   
end

end