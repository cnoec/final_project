function [x_kp1, fx_kp1, i] = linesearch(fnc, grad_fxk, x_k, p_k, tk_max, beta, c, n_iter_max)
%   [xkp1, fxkp1, n_iter] = linesearch(fnc, grad_fxk, xk, pk, tk_max, beta, c, n_iter_max)

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