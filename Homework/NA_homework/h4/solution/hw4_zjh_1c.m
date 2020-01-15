clc;clear all;close all;
a = -5;b = 5;n = 8;
syms x;syms y;
q = zeros(n,1);q = sym(q);
c = zeros(n,1);c = sym(c);
f = 1/(1+y^2);
q0 = sqrt(1/(b-a));
c0 = int(f*q0,y,a,b);
p=c0*q0;
for i=1:n
    q(i) = gram_schmidt(i,x,y,a,b);
    c(i) = int(f*q(i),y,a,b);
    p = p+c(i)*q(i);
end
%output the ploynomial p(y)
p(y) = p
%plot f(y)
fplot(y,f,"b");
hold on
%plot p(y)
fplot(y,p,"r--");
function [q] = gram_schmidt(n,x,y,a,b)
q = sqrt((2*n+1)/(b-a))/(2^n*factorial(n))*diff((x^2-1)^n,x,n);
x = 2*(y-a)/(b-a)-1;
q = eval(q);
end
