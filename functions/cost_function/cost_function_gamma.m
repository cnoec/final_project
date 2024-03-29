function [FUNCTION_VALUE, point_index ,xi] = cost_function_gamma(u, xi0, T_end, Ts, waypoints, n_wp, gamma ,delta,str)
% Cost function that is used in step 2 and step 3 for unconstrained and 
% constrained optimization.
%  
%   INPUTS:
%              u          =    input
%              xi0        =    initial conditions  
%              T_end      =    simulation time
%              Ts         =    time step for the steering sampling
%              waypoints  =    vector of the point to follow
%              n_wp       =    number of waypoints decided by the user
%                              in the main file
%              gamma      =    weigth of the penalty for the space traveled
%              delta      =    tuning parameter for cost function
%
%   OUTPUTS:
%              SUM { ||P(i) - (X,Y)||^2 }  +  gamma * {SUM distances }

% initialization
Ts_sim              =       1e-2;
n_states            =       T_end/Ts_sim;
n_wp                =       length(waypoints);

[xi,~,~,~]          =       trajectory_generation_cc(u, xi0, T_end, Ts, Ts_sim,str);


% definition of Px Py
Px                  =       ones(n_states, n_wp);
Py                  =       ones(n_states, n_wp);
for i = 1:n_wp
    Px(:,i)         =       waypoints(i,1);
    Py(:,i)         =       waypoints(i,2);
end

% distance computation
result              =       zeros(n_wp,1);
point_index         =       zeros(n_wp,1);

for i = 1:n_wp
    a               =       xi(1,:)'-Px(:,i);
    b               =       xi(2,:)'-Py(:,i);
    [result(i,1),point_index(i)] = min(a.^2+b.^2);
end

% the last P(i) is forced to be the last simulated point:
result(n_wp)        =       (xi(1,end)-waypoints(end,1))^2+(xi(2,end)-waypoints(end,2))^2;
point_index(n_wp)   =       n_states;

% penalty computation
penalty             =       zeros(n_states-1,1);

for i = 1:n_states-1
    penalty(i)      =       sqrt((xi(1,i)-xi(1,i+1))^2 + (xi(2,i)-xi(2,i+1))^2);   
end

% final value of the function
sum_res             =       sum(result);
sum_weight          =       sum(penalty);

FUNCTION_VALUE      =       (sum_res + gamma*sum_weight)/delta;

end