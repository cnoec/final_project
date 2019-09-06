function T1_val = merit_function( fun,xk,A,b,C,d,p,q,sigma,tau,mode )
% MERIT_FUNCTION Computes the value of the merit function.
%
%   INPUTS:
%           fun         =   cost function
%           xk          =   current point
%           A           =   matrix defining linear equality constraints
%           b           =   vector defining linear equality constraints
%           C           =   matrix defining linear inequality constraints
%           d           =   vector defining linear inequality constraints
%           p           =   number of nonlinear equality constraints
%           q           =   number of nonlinear inequality constraints
%           sigma       =   vector of weights, one for each equality
%                           constraint
%           tau         =   vector of weights, one for each inequality
%                           constraint
%           mode        =   'GN': Gauss-Newton
%                           'BFGS': BFGS  
%   OUTPUTS:
%           T1_val      =   value of the merit function

[Vk]                =   fun(xk);

if strcmp( mode,'BFGS' )
    
    fxk             =   Vk( (1:(end-p-q)),1 );
    
end

if(~isempty( A )) %This if is needed to understand if we also have linear eq. constraints
        
        gxk         =   [A*xk-b;
                         Vk(end-p-q+1:end-q,1)];
        
else
        
        gxk         =   Vk(end-p-q+1:end-q,1);
        
end
    
if(~isempty( C )) %This if is needed to understand if we also have linear ineq. constraints
        
        hxk         =   [C*xk-d;
                         Vk(end-q+1:end,1)];
        
else
        
        hxk         =   Vk(end-q+1:end,1);
        
end
    
    T1_val          =   fxk+sigma'*abs( gxk )+tau'*max( zeros( size( hxk ) ), -hxk );


end % END of the function

