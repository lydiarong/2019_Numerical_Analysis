syms x n;
L=x;
order=10;
for num=0:order
    y=(x^2-1)^n;%(1/((2^n)*factorial(n)))*
    res=(1/((2^n)*factorial(n)))*(diff(subs(y,n,num),x,num));
    L(num+1)=subs(res,n,num);
end
L
z=ones(3,3)*x;
for i=1:order
    for j=1:order      
         z(i,j)=int(L(i)*L(j),x,[-1,1]);
    end
end
% z 
% z(1,1)=0;
% z(2,2)=0;
% z(3,3)=0;
z