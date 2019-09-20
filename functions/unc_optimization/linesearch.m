function [x_kp1, fx_kp1, i] = linesearch(fnc, grad_fxk, x_k, p_k, tk_max, beta, c, n_iter_max)
% Computes the decision variables' value x_kp1 at iteration k+1
% by performing a back-tracking line search. It attempts to have a 
% sufficient decrease of the cost function f(x_k+tk*p_k) where p_k is the
% search direction. p_k must be locally a descent direction (i.e. gradfxk'*pk<0).
%
%   INPUTS:
%           fnc         =   cost function
%           grad_fxk    =   gradient of the cost function with respect to
%                           x, evaluated at xk
%           x_k         =   value of the decision variables at the current
%                           iteration
%           p_k         =   search direction
%           tk_max      =   maximum step size
%           beta        =   ratio tk_ip1/tk_i during the back-tracking line
%                           search (beta in (0,1))
%           c           =   ratio between acheived decrease and decrease
%                           predicted by the first-order Taylor expansion,
%                           sufficient to exit the back-tracking line
%                           search algorithm
%           nitermax    =   maximum number of iterations
%
%   OUTPUTS:
%           x_kp1       =   obtained value of xk+tk*pk
%           fx_kp1      =   cost function evaluated at xkp1
%           i           =   number of iterations employed to satisfy Armijo
%                           condition

t_i = tk_max;
i=0;
fx = fnc(x_k);
x_kp1 = x_k+t_i*p_k;
fx_kp1 = fnc(x_kp1);

while( (fx_kp1 > fx + t_i*c*grad_fxk'*p_k) && (i<n_iter_max))
    t_i = beta*t_i;
    i=i+1;
    x_kp1 = x_k+t_i*p_k;
    fx_kp1 = fnc(x_kp1);
end