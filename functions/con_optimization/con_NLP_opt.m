function [xstar,fxstar,k,exitflag,xsequence] = con_NLP_opt(fun,x0,A,b,C,d,p,q,con_options,filename)
% COS_NLP_OPT Attempts to solve the problem:
%                   min f(x)  
%                     s.t.
%                   A*x = b
%                  C*x >= d
%                  g(x) = 0
%                 h(x) >= 0
% and, if successful, returns a local minimizer xstar and the related local
% optimum fxstar=f(xstar). The solver employs a Sequential Quadratic programming
% optimization scheme and back-tracking line search with l-1 norm merit function
% and Armijo condition.
%
%   INPUTS:
%           fun         =   function providing the scalar cost function value 
%                           f(x) and the vectors of p nonlinear equality
%                           constraints g(x), g:R^n->R^p, and q nonlinear 
%                           inequality constraints h(x)
%           x0          =   initial guess for the optimization variables
%           A           =   matrix defining linear equality constraints
%           b           =   vector defining linear equality constraints
%           C           =   matrix defining linear inequality constraints
%           d           =   vector defining linear inequality constraints
%           p           =   number of nonlinear equality constraints
%           q           =   number of nonlinear inequality constraints
%           con_options =   optimization options prepared with myoptimset
%                           command
%           filename    =   string defining the name of the file in which
%                           the user desires to print the iterations of the 
%                           optimization routine (it is done in order to 
%                           speed up the op. routine)
%                               - Use "filename" to obtain filename.txt
%                               - Use 1 to print all the iterations on the
%                                 command window
%
%   OUTPUTS:
%           xstar       =   exit value, either a local minimizer or the
%                           value of x at the last iterate
%           fxstar      =   cost function evaluated at xstar
%           niter       =   number of employed iterations 
%           exitflag    =   termination condition:
%                           -2: max iterations reached, unfeasible point
%                           -1: max. number of iterations reached
%                            1: local minimum possible, gradient condition
%                            2: local minimum possible, step size condition
%                            3: local minimum possible, cost decrease condition
%           xsequence   =   sequence of iterations {xk} (only if option
%                           xsequence is set to 'on')

%% Initialization

n                   =   length(x0);
k                   =   0;
deltaxk_rel         =   1;
deltaf_rel          =   1;


if isstring(filename)
    
    F               =   fopen(filename+".txt",'w');
    

elseif (filename == 1)
    
    F               =   filename;
    
end

% Note that we need a lambda for each equality contraint, which means that
% we have to consider p nonlinear constraints plus the linear ones
lambdak             =   zeros( p + size(A,1), 1 );
sigmak              =   zeros( p + size(A,1), 1 );

% Note that we need a mu for each equality contraint, which means that
% we have to consider q nonlinear constraints plus the linear ones
muk                 =   zeros( q + size(C,1), 1 );
tauk                =   zeros( q + size(C,1), 1 );

eq_con_max          =   0;
ineq_con_min        =   0;


if ~isempty(con_options.outputfcn)
    
    outputfun       =   con_options.outputfcn;
    
end


if ( strcmp(con_options.display,'Iter') )
    
     fprintf(F,'Iteration       NormGrad          Cost      Equality     Inequality     Rel. cost         Rel. x     Line-search\r');
    
end


%Start sequence of optimization variables at each iteration
xsequence           =    [];

if strcmp(con_options.xsequence,'on')
    
    xsequence       =    [xsequence, x0];
    
end


%% Iterations - BFGS method

%Cost and constraints computation + their gradients
xk              =   x0;
Hk              =   1e-4*eye(n);
[Vk,gradVk]     =   gradient_num_comp(fun,x0,con_options.gradmethod,con_options.graddx);
fxk             =   Vk(1:end-p-q,1); %Note that as input this algorithm 
                                     %takes the cost function and the 
                                     %nonlinear functions defining inequality and
                                     %equality constraints, so we need
                                     %to assign to the cost function
                                     %only the cost function part.
gradfxk         =   gradVk(:,1:end-p-q);

if(~isempty( A )) %This if is needed to understand if we also have linear eq. constraints

    gxk         =   [A*xk-b;
                     Vk(end-p-q+1:end-q,1)];
    gradgk      =   [A',gradVk(:,end-p-q+1:end-q,1)];

else

    gxk         =   Vk(end-p-q+1:end-q,1);
    gradgk      =   gradVk(:,end-p-q+1:end-q,1);

end

if(~isempty( C )) %This if is needed to understand if we also have linear ineq. constraints

    hxk         =   [C*xk-d;
                     Vk(end-q+1:end,1)];
    gradhk      =   [C',gradVk(:,end-q+1:end,1)];

else

    hxk         =   Vk(end-q+1:end,1);
    gradhk      =   gradVk(:,end-q+1:end,1);

end

% Lagrange function gradient computation
gradLagr        =   gradfxk-gradgk*lambdak-gradhk*muk;

% Worst-case equality and inequality constraints computation (for feedback and termination conditions)
if ~isempty(gxk)

    eq_con_max  =   max( abs( gxk ) );

end

if ~isempty(hxk)

    ineq_con_min=   min(hxk);

end

% Feedback and output function
if strcmp(con_options.display,'Iter')
    if ineq_con_min<=0 && sign(ineq_con_min)==-1
        fprintf(F,'%9.0f    %7.5e   %6.5e   %6.5e   %6.5e   %6.5e    %6.5e            %4.0f\r',...
            k,norm(gradLagr),fxk,eq_con_max,ineq_con_min,deltaf_rel,deltaxk_rel,0);
    else
        fprintf(F,'%9.0f    %7.5e   %6.5e   %6.5e    %6.5e   %6.5e    %6.5e           %4.0f\r',...
            k,norm(gradLagr),fxk,eq_con_max,ineq_con_min,deltaf_rel,deltaxk_rel,0);
    end
end

%Store sequence of optimization variables at each iteration
if ~isempty(con_options.outputfcn)

    outputfun(xk);

end

%Iterations
while  norm(gradLagr)                   > con_options.tolgrad   &&  ...
       k                                < con_options.nitermax  &&  ...
       deltaxk_rel                      > con_options.tolx      &&  ...
       deltaf_rel                       > con_options.tolfun    ||  ...
       (max(eq_con_max,-ineq_con_min)   > con_options.tolconstr)&&k< con_options.nitermax    %End

    %Compute the new search direction
    [pk,~,exitflag,~,LagMult]      =   quadprog(Hk,gradfxk,-gradhk',hxk,gradgk',...
                                         -gxk,[],[],[],con_options.QPoptions);
                                     
    lambda_tilde            =   -LagMult.eqlin;
    mu_tilde                =   LagMult.ineqlin;
    delta_lambda            =   lambda_tilde - lambdak;
    delta_mu                =   mu_tilde - muk;

    %Update merit function weights
    sigmak                  =   max(abs(lambda_tilde),(sigmak+abs(lambda_tilde))/2);
    tauk                    =   max(abs(mu_tilde),(tauk+abs(mu_tilde))/2);
    T1_fun                  =   @(x)merit_function( fun,x,A,b,C,d,p,q,...
                                                    sigmak,tauk,'BFGS' );
    T1k                     =   T1_fun(xk);

    %Directional derivative computation
    ind_h_violated          =   hxk<=0; %We save the positions of violated
                                        %inequality constraints
    gradhk_violated         =   gradhk(:,ind_h_violated);
    tauk_violated           =   tauk(ind_h_violated,1);
    DT1k                    =   gradfxk'*pk-sigmak'*abs( gxk )-...  %In this way we'll obtain a directional derivative 
                                tauk_violated'*gradhk_violated'*pk; %which weights also the violated constraints

    %Line search with the merit function
    [xkp1,fxkp1,niter_LS,tk]=   LS_merit_function(T1_fun,T1k,DT1k,xk,pk,...
                                                  con_options.ls_tkmax,...
                                                  con_options.ls_beta,...
                                                  con_options.ls_c,...
                                                  con_options.ls_nitermax);
    deltaxk_rel             =   norm(xkp1-xk)/max(eps,norm(xk));
    deltaf_rel              =   abs(fxkp1-fxk)/max(eps,abs(fxk));
    lambdakp1               =   lambdak+tk*delta_lambda;
    mukp1                   =   muk+tk*delta_mu;

    %NEW Cost and constraints computation + their gradients
    [Vkp1,gradVkp1] =   gradient_num_comp(fun,xkp1,con_options.gradmethod,con_options.graddx);
    fxkp1           =   Vkp1(1:end-p-q,1); %Note that as input this algorithm 
                                           %takes the cost function and the 
                                           %nonlinear functions defining inequality and
                                           %equality constraints, so we need
                                           %to assign to the cost function
                                           %only the cost function part.
    gradfxkp1       =   gradVkp1(:,1:end-p-q);

    if(~isempty( A )) %This if is needed to understand if we also have linear eq. constraints

        gxkp1       =   [A*xkp1-b;
                         Vkp1(end-p-q+1:end-q,1)];
        gradgkp1    =   [A',gradVkp1(:,end-p-q+1:end-q,1)];

    else

        gxkp1       =   Vkp1(end-p-q+1:end-q,1);
        gradgkp1    =   gradVkp1(:,end-p-q+1:end-q,1);

    end

    if(~isempty( C )) %This if is needed to understand if we also have linear ineq. constraints

        hxkp1       =   [C*xkp1-d;
                         Vkp1(end-q+1:end,1)];
        gradhkp1    =   [C',gradVkp1(:,end-q+1:end,1)];

    else

        hxkp1       =   Vkp1(end-q+1:end,1);
        gradhkp1    =   gradVkp1(:,end-q+1:end,1);

    end

    % NEW Lagrange function gradient computation
    gradLagrkp1     =   gradfxkp1-gradgkp1*lambdakp1-gradhkp1*mukp1;

    % Lagrange function gradient computation with previous xk and new
    % multipliers (for BFGS update)
    gradLagrk_kp1   =   gradfxk-gradgk*lambdakp1-gradhk*mukp1;

    % Update Hessian estimate with BFGS rule
    y               =   gradLagrkp1-gradLagrk_kp1;
    s               =   xkp1-xk;

    if max(abs(s))>0

        if y'*s<= con_options.BFGS_gamma*(s'*Hk*s)

            y       =   y+(con_options.BFGS_gamma*s'*Hk*s-s'*y)/(s'*Hk*s-s'*y)*(Hk*s-y);

        end

        Hk          =   Hk-(Hk*(s*s')*Hk)/(s'*Hk*s)+(y*y')/(s'*y);
        Hk          =   0.5*(Hk+Hk');

    end

    % Update variables
    k               =   k+1;
    xk              =   xkp1;
    fxk             =   fxkp1;
    gradfxk         =   gradfxkp1;
    gxk             =   gxkp1;
    gradgk          =   gradgkp1;
    hxk             =   hxkp1;
    gradhk          =   gradhkp1;
    gradLagr        =   gradLagrkp1;

    % Worst-case equality and inequality constraints computation (for feedback and termination conditions)
    if ~isempty(gxk)

        eq_con_max  =   max( abs( gxk ) );

    end

    if ~isempty(hxk)

        ineq_con_min=   min(hxk);

    end

    % Feedback and output function
    if strcmp(con_options.display,'Iter')
        if ineq_con_min<=0 && sign(ineq_con_min)==-1
            fprintf(F,'%9.0f    %7.5e   %6.5e   %6.5e   %6.5e   %6.5e    %6.5e            %4.0f\r',...
                k,norm(gradLagr),fxk,eq_con_max,ineq_con_min,deltaf_rel,deltaxk_rel,niter_LS);
        else
            fprintf(F,'%9.0f    %7.5e   %6.5e   %6.5e    %6.5e   %6.5e    %6.5e           %4.0f\r',...
                k,norm(gradLagr),fxk,eq_con_max,ineq_con_min,deltaf_rel,deltaxk_rel,niter_LS);
        end
    end

    if ~isempty(con_options.outputfcn)

        outputfun(xk);

    end

    % Store sequence of optimization variables at each iteration
    if strcmp(con_options.xsequence,'on')

        xsequence       =   [xsequence, xk];

    end


end %END of the while

%% Termination

xstar   =   xk;
fxstar  =   fxk;
if max(eq_con_max,-ineq_con_min) <= con_options.tolconstr
    
    if norm(gradLagr) <= con_options.tolgrad
        
        exitflag    =   1;
        
        if strcmp(con_options.display,'Iter')
            
            fprintf(F,'Local minimum possible, directional derivative smaller than tolerance. Constraints satisfied.\r');
        
        end
        
    elseif k >= con_options.nitermax
        
        exitflag    =   -1;
        
        if strcmp(con_options.display,'Iter')
            
            fprintf(F,'Maximum number of iterations reached. Constraints satisfied.\r');
            
        end
        
    elseif deltaxk_rel <= con_options.tolx
        
        exitflag    =   2;
        
        if strcmp(con_options.display,'Iter')
            
            fprintf(F,'Local minimum possible, relative step size smaller than tolerance. Constraints satisfied.\r');
            
        end
        
    elseif deltaf_rel <= con_options.tolfun
        
        exitflag    =   3;
        
        if strcmp(con_options.display,'Iter')
            
            fprintf(F,'Local minimum possible, relative cost decrease smaller than tolerance. Constraints satisfied.\r');
        
        end
        
    end
    
elseif max(eq_con_max,-ineq_con_min) > con_options.tolconstr
    
    if k >= con_options.nitermax
        
        exitflag    =   -2;
        
        if strcmp(con_options.display,'Iter')
            
            fprintf(F,'Maximum number of iterations reached. Constraints not satisfied.\r');
            
        end
        
    end
    
end


%%

if isstring(filename)
    
    fclose(F);
    
end

end %END of the function

