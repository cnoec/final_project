function [xi, t_vec, end_check] = trajectory_generation(u, xi_0, T_end, Ts)
% Generation of the trajectory, given an input and a starting condition. 
% It simulates with a constant step (Forward Euler) up to T_end-Ts. The
% inputs are meant as before the torques and then the steering angles. 
%
%   INPUTS: 
%          u         =   input sequence to generate the trajectory 
%          xi_0      =   initial conditions
%          T_end     =   simulation time
%          Ts        =   time step for the numerical simulation
%                     
%   OUTPUTS:
%          xi        =   state of the system wrt time
%          t_vec     =   time vector
%          end_check =   input dimensionality check. it returns:
%                        1 -> ok
%                        0 -> input with wrong dimensions

% Time vector
end_check           =       1;
t_vec               =       0:Ts:(T_end-Ts);
t_vec               =       t_vec';
N                   =       length(t_vec); 

% Dimensional check
if 2*N ~= length(u)
    
    end_check       =       0;
    xi              =       0;
    return
    
end

% Initialization
T_in                =       u(1:N);
delta_in            =       u((N+1):(2*N));

xi                  =       zeros(6,N);
xi(:,1)             =       xi_0;

run('Parameters.m');

% Simulation with forward Eulero

for ind=2:N
    
    input           =       [T_in(ind); delta_in(ind);];
    xi(:,ind)       =       xi(:,ind-1)  + Ts*Vehicle_Model_Function( xi(:,ind-1), input, theta);
    
end






