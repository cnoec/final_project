function options = unc_optimalset
%% save options

options.debug = 'on';
options.display = 'on';

%%  tolerances

options.tol_grad    	=	1e-6;       % Termination tolerance on the norm                                        % of the directional derivative
options.tol_x          =	1e-12;      % Termination tolerance on the relative
options.tol_fun        =	1e-12;      % Termination tolerance on the relative
options.niter_max      =	1000;       % Termination tolerance on the number of

%% gradient computation options

options.gradmethod  	=	'CD';

% FD = Forward Differences
% CD = Central Differences
% IP = Imaginary part

options.grad_dx        =	2^-17;

% use 2e-26 for FD and 2e-17 for CD
                                        
%% linesearch options
options.ls_tkmax      =	1;          % max step size
options.ls_beta       =	0.8;        % scaling factor
options.ls_c          =	0.1;        % factor for linear approx 
options.ls_nitermax   =	300;        % max number of iterations

%% Quasi-Newton method options

options.BFGS_gamma  	=	1e-1;       % gamma factor for Powell's trick in
options.eps = 10e-16;

end


