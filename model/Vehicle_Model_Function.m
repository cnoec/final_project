function [xi_dot] = Vehicle_Model_Function(xi,u,theta)
% Nonlinear dynamic model of a road vehicle with six states: 2-D position,
% 2-D velocity, yaw angle and yaw rate. Nonlinear lateral tyre forces with
% Fiala model; saturation on braking and driving torque, saturation on 
% front wheel steering angle. Partial de-coupling of longitudinal and
% lateral dynamics (assume longitudinal speed varies slowly with respect to
% lateral dynamics)
%
% Inputs:   tau     (time - for use with ode45)
%           xi      (model state)
%           u       (braking/driving torque and steering angle)
%           d       (lateral wind),
%           
%           theta   (model parameters)
%
% Outputs:  xi_dot  (derivative of the state with respect to time)
%           Forces  (longitudinal and lateral forces)

%% Read states

g = 9.81;

X       = xi(1,1);          % inertial X position (m)                 
Y       = xi(2,1);          % inertial Y position (m)
Ux      = xi(3,1);          % body x velocity (m/s)
beta    = xi(4,1);          % sideslip angle (rad)
psi     = xi(5,1);          % yaw angle (rad)
r       = xi(6,1);          % yaw rate (rad/s)

%% Read inputs

Td      = u(1,1);           % driving/braking torque (N*m)
delta   = u(2,1);           % front wheel steering angle (rad)
W       = 0;           % lateral wind speed (m/s)

%% Parameter vector

m       = theta(1,1);       % vehicle mass (kg)
Jz      = theta(2,1);       % vehicle moment of inertia (kg*m^2)
a       = theta(3,1);       % distance between center of gravity and front axle (m)
b       = theta(4,1);       % distance between center of gravity and rear axle (m)
Cf      = theta(5,1);       % front axle cornering stiffness (N/rad)
Cr      = theta(6,1);       % rear axle cornering stiffness (N/rad)
rw      = theta(7,1);       % wheel radius (m)
mu      = theta(8,1);       % road friction coefficient
Af      = theta(9,1);       % vehicle front surface (m^2)
Al      = theta(10,1);      % vehicle lateral surface (m^2)
Cx      = theta(11,1);      % vehicle front aerodynamic drag coefficient
Rr      = theta(12,1);      % rolling resistance coefficient(N*s/m)
rho     = theta(13,1);      % air density (kg/m^3)


%% Equations

Uy      = tan( beta )*Ux;

% Tire slip angles
alphaf  = atan( (Uy + a*r)/(Ux) ) - delta;
alphar  = atan( (Uy - b*r)/(Ux) );

% Terms to compute the "FIALA MODEL"
zf      = tan(alphaf);
zr      = tan(alphar);

%% Forces

% Vertical Loads
Fzf     = m*g*( b/(a+b) );
Fzr     = m*g*( a/(a+b) );

% Lateral Forces
Fyf     = min( mu*Fzf, max( -mu*Fzf,-Cf*zf + ( Cf^2*abs(zf)*zf )/( 3*mu*Fzf ) - (Cf^3*zf^3)/(27*mu^2*Fzf^2) ) );
Fyr     = min( mu*Fzr, max( -mu*Fzr,-Cr*zr + ( Cr^2*abs(zr)*zr )/( 3*mu*Fzr ) - (Cr^3*zr^3)/(27*mu^2*Fzr^2) ) );

% Aerodynamic Lateral Force
Fyd     = (1/2)*rho*Al*W^2;

% Force acting along the local x direction (on the rear wheel)
Fx      = Td/rw;

% Rolling resistance Force
Fr      = Rr*Ux;

% Aerodynamic longitudinal Force
Fxd     = (1/2)*rho*Af*Cx*Ux^2;

%% State equations

X_dot        = Ux*cos(psi) - Uy*sin(psi);
Y_dot        = Ux*sin(psi) + Uy*cos(psi);
Ux_dot       = ( (Fx - Fyf*sin(delta)) - Fr - Fxd )/( m );
beta_dot     = ( Fyf*cos(delta) + Fyr + Fyd )/( m*Ux ) - r;
psi_dot      = r;
r_dot        = ( a*Fyf*cos(delta) - b*Fyr )/( Jz );

xi_dot(1,1)  = X_dot;
xi_dot(2,1)  = Y_dot;
xi_dot(3,1)  = Ux_dot;
xi_dot(4,1)  = beta_dot;
xi_dot(5,1)  = psi_dot;
xi_dot(6,1)  = r_dot;


end

