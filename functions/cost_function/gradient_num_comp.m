function [y,gradient] = gradient_num_comp(fnc, xkr, method, dx)
% Computes the gradient of a given function, using one of several possible
% methods.
%   INPUTS:
%           fnc         =   its gradient shall be evaluated. usually a
%                           vector with length N
%           xkr         =   value of the input argument where to evaluate
%                           the gradient (dfun(x)/dx)'
%           method      =   string indicating the differentiation method: 'FD' 
%                           (Forward Finite Differences), 'CD' (Central
%                           Finite Differences), 'UP' (User provided), 
%                           'IM' (Imaginary-part trick).
%           dx          =   perturbation step used in Finite Difference
%                           approximation
%
%   OUTPUTS:
%           y           =   value of fnc(xk)
%           gradient    =   n-by-N matrix with in each column the
%                           partial derivatives of the corresponding element 
%                           of function fun(x) with respect to each 
%                           component of x, evaluated at xk

% Initialization 
n = length(xkr);
N = length(fnc(xkr));
gradient = zeros(n,N);
p = zeros(n,1);
    

% Gradient computation with different methods
if strcmp(method, 'FD')

    p(1)=dx;
    gradient(1,:) = (fnc(xkr+p)-fnc(xkr)).'/dx;
    
    for ind = 2:n
        p(ind)=dx;
        p(ind-1)=0;
        gradient(ind,:) = (fnc(xkr+p)-fnc(xkr)).'/dx;    
    end
    
    
elseif strcmp(method, 'CD')
    p(1)=dx;
    gradient(1,:) = (fnc(xkr+p)-fnc(xkr-p))/(2*dx);
    
    for ind=2:n
        p(ind)=dx;
        p(ind-1)= 0;
        gradient(ind,:) = (fnc(xkr+p)-fnc(xkr-p))/(2*dx);
    end
    
elseif strcmp(method, 'IP')
    t = 1e-100;
    i = sqrt(-1);
    
    p(1)=1;
    gradient(1,:) = imag(fnc(xkr+i*t*p))/t;
    
    for ind=2:n
        p(ind)=1;
        p(ind-1)=0;
        gradient(ind,:) = imag(fnc(xkr+i*t*p))/t;
    end
end

y=fnc(xkr);




















end

