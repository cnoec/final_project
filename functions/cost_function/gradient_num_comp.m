function [y,gradient] = gradient_num_comp(fnc, xkr, method, dx)
    %IT COMPUTE THE GRADIENT OF THE FUNCTION (TRANSPOSE OF THE JACOBIAN)
    %[y, gradient] = gradient_num_comp(fnc, xkr, method, dx)
    
    n = length(xkr);
    N = length(fnc(xkr));
    gradient = zeros(n,N);
    p = zeros(n,1);
    
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

