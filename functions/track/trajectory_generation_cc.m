function [xi, t_vec, end_check,Torque] = trajectory_generation_cc(u, xi_0, T_end, Ts_steer, Ts_sim,str)
% trajecoty generation with cruise control, that sets the speed references.
% in this function a downsampling is done.
%
% INPUTS:  
%               u         =     input 
%               xi_0      =     initial conditions
%               T_end     =     simulation time
%               Ts_steer  =     sampling time for the steering
%               Ts_sim    =     sampling time for the simulation
%                     
% OUTPUTS: 
%               xi        =     state of the system wrt time
%               t_vec     =     time vector
%               end_check =     input dimensionality check. it returns:
%                               1 -> ok
%                               0 -> input with wrong dimensions
%               Torque    =     torque vector
%       

% Time vector and initialization
end_check           =       1;
t_vec               =       0:Ts_sim:(T_end-Ts_sim);
t_vec               =       t_vec';
N                   =       length(t_vec);
downsampling        =       Ts_steer/Ts_sim;
n_ref               =       2;

if strcmp (str,"yes")  
    u_1             =       zeros(length(u)-1,1);
    u_1(1:end)      =       u(1:end-1);
    u               =       u_1;
    
    clear u_1    
end

% downsampling
for ind=1:(T_end/Ts_steer)
    steer(((ind-1)*downsampling+1):ind*downsampling) = u(n_ref+ind);  
end

steer               =       steer.';

% Dimensional check
if  length(steer) ~= length(t_vec) 
    end_check      =       0;
    xi             =       0;
    disp('Dimensional Error!');
    return 
end

% Parameters
xi                  =       zeros(6,N);
xi(:,1)             =       xi_0;

load('parameters.mat');

global num_dyn den_dyn num_int

T_dyn               =       0;
T_int               =       0;
ek                  =       u(1)-xi_0(3);
ekm1                =       ek;

% simulation

for ind=2:N 
    % Control action
    T_int           =       min(Tdmax, max(Tdmin, T_int+num_int*[ek; ekm1]));
    T_dyn           =       -den_dyn(1,2)*T_dyn + num_dyn*[ek; ekm1];
    T_in            =       T_int + T_dyn;
    
    % Saturation
    T_in            =       min(Tdmax, T_in);
    T_in            =       max(Tdmin, T_in);
    Torque(ind-1)   =       T_in;
    
    % model integration
    input           =       [T_in; 
                            steer(ind);];
    xi(:,ind)       =       xi(:,ind-1)  + Ts_sim*Vehicle_Model_Function(xi(:,ind-1), input, theta);
    
    % error update
    ekm1            =       ek;
    if ind < floor(N/2)   
        ek          =       u(1) - xi(3,ind);     
    else
        ek          =       u(2)- xi(3,ind);
    end
end