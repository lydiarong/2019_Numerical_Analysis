function [m,a]=LU(a,n)
m = zeros(n,n);             % initial the n*n matrix m zeros 
for i = 1:n;     m(i,i) = 1;  end   % let the diagonal elements be 1 
for j = 1 : n-1
    if abs(a(j,j))<eps; 
        error('zero pivot encountered');    % when the zero pivot happens,end the process
    end
    for i = j+1 : n
        mult = a(i,j)/a(j,j);
        m(i,j) = mult;  
        for k = j:n
            a(i,k) = a(i,k) - mult*a(j,k);
        end
    end
end

disp('  L=');  disp(m);
disp('  U=');  disp(a);
disp('  LU='); disp(m*a);