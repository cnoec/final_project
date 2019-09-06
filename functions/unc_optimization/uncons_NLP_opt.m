function [x_opt, exit_flag, debug_struct] = uncons_NLP_opt(fnc, x0, options)
% Implmentation of NLP unconstrained problem

%% general initialization

x_opt = 0;
exit_flag = 0;
if strcmp(options.display,'on')
    F = fopen('unc_iterations.txt','w');
    fprintf(F,'Iteration       NormGrad          Cost      Rel. cost         Rel. x      Line-search interations\r');
end

delta_x = 1;
delta_f = 1;
n = length(x0);
n_iter=0;
x_k=x0;

%% BFGS related initialization

gamma = options.BFGS_gamma;
[fx,grad] = gradient_num_comp(fnc, x_k, options.gradmethod, options.grad_dx);
H_k=eye(n)*1e-4;
p_k = -H_k\grad;

if strcmp(options.display, 'on')
    fprintf(F,'%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f   (INITIAL GUESS)\r',n_iter,norm(grad'*p_k),fx,delta_f,delta_x,0);
end

%% debug struct creation

if strcmp(options.debug, 'on')      %pre-allocation of variable to avoid changing size at each iteration
    debug_struct.seq = zeros(n,options.niter_max);
    debug_struct.ls_iter = zeros(options.niter_max,1);
    debug_struct.grad_norm = zeros(options.niter_max,1);
    debug_struct.cost = zeros(options.niter_max,1);
    debug_struct.rel = zeros(2,options.niter_max);
end

%% algorithm

while(norm(grad'*p_k) > options.tol_grad && delta_x > options.tol_x && delta_f > options.tol_fun && n_iter < options.niter_max) 
    % step 4: linesearch    
    n_iter = 1+n_iter;
    [x_kp1, fx_kp1, ls_iter] = linesearch(fnc, grad, x_k, p_k, options.ls_tkmax, options.ls_beta, options.ls_c, options.ls_nitermax);
   
    % step 5: Hessian approximation:
    [~,grad_kp1] =  gradient_num_comp(fnc, x_kp1, options.gradmethod, options.grad_dx);
    y = (grad_kp1 - grad);
    s = (x_kp1 - x_k);

    % Powell trick to enforce def pos
    
    if y.'*s <= gamma*s.'*H_k*s
        lambda = (gamma*s.'*H_k*s - s.'*y)/(s.'*H_k*s-s.'*y);
        y = y + lambda*(H_k*s-y);
    end

    H_k = H_k - H_k*s*(s.')*H_k/((s.')*H_k*s) + (y*(y.'))/((s.')*y);

    % step 6: tolerance update
    delta_x = norm(x_kp1 - x_k,2)/max(eps, norm(x_k,2));
    delta_f = norm(fx_kp1 - fx)/max(eps, norm(fx,2));
    
    % step 7: updating
    x_k = x_kp1;
    fx = fx_kp1;
    
    grad = grad_kp1;
    p_k = -H_k\grad;
    
    if strcmp(options.debug, 'on')
        debug_struct.seq(:,n_iter) = x_k;
        debug_struct.ls_iter(n_iter) = ls_iter;
        debug_struct.grad_norm(n_iter) = norm(grad'*p_k);
        debug_struct.cost(n_iter) = fx;
        debug_struct.rel(:,n_iter) = [delta_f; delta_x];
    end

    if strcmp(options.display, 'on')
        fprintf(F,'%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',n_iter,norm(grad'*p_k),fx,delta_f,delta_x,ls_iter);
    end
end

%% results

x_opt = x_k;

if strcmp(options.debug,'on')
    debug_struct.niter = n_iter;
    debug_struct.seq(:,(n_iter+1):options.niter_max) = [];
    debug_struct.ls_iter((n_iter+1):options.niter_max) = [];
    debug_struct.cost((n_iter+1):options.niter_max) = [];
    debug_struct.grad_norm((n_iter+1):options.niter_max) = [];
    debug_struct.rel(:,(n_iter+1):options.niter_max) = []; 
else
    debug_struct = [];
end
%% exit reason

if norm(grad'*p_k) > options.tol_grad
    exit_flag = 1;
elseif delta_x > options.tol_x
    exit_flag = 2;
elseif delta_f > options.tol_fun
    exit_flag = 3;
elseif n_iter < options.niter_max
    exit_flag = 4;
end

if strcmp(options.display,'on')
    fclose(F);
end
