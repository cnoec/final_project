function [xi, t_vec, end_check] = trajectory_generation(u, xi_0, T_end, Ts)
%       Input:  u: input (torque, delta)
%               xi_0: initial conditions
%               T_end: simulation time
%               Ts: time step for the numerical simulation
%                     
%       Output: xi: state of the system wrt time
%               t_vec: time vector
%               end_check:  1: ok
%                           0: input di dimensione errata
%       
%       Questa funzione simula a passo costante (eulero in avanti) dati
%       ingressi e tempo di simulazione, fino a T_end-Ts.
%
%       NOTA BENE: L'INPUT E' INTESO COME PRIMA TUTTE LE COPPIE E DOPO
%       TUTTI GLI ANGOLI, DITEMI SE E' DA CAMBIARE

%% vettore del tempo

% path        =       pwd;
% addpath( '..\Model' );
% addpath( '..' );

end_check = 1;
t_vec = 0:Ts:(T_end-Ts);
t_vec = t_vec';
N = length(t_vec); 

%% dimensional check

if 2*N ~= length(u)
    end_check =  0;
    xi=0;
    return
end

%% inizializzazione

T_in = u(1:N);
delta_in = u((N+1):(2*N));

tau                 =       0;
d                   =       0;

xi = zeros(6,N);
xi(:,1) = xi_0;

run('Parameters.m');

%% simulazione

for ind=2:N
    input = [T_in(ind); delta_in(ind);];
    xi(:,ind) = xi(:,ind-1)  + Ts*Vehicle_Model_Function(tau, xi(:,ind-1), input, d, theta);
end






