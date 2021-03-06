clear all;close all;
syms x y;
n=3;

for i=1:n
a(i)=x^(i-1)
end
qb(1)=a(1);
q(1)=qb(1)/sqrt(int(qb(1)*qb(1),x,[-1,1]));
qb(2)=(a(2)-int(q(1)*a(2),x,[-1,1])*q(1));
q(2)=qb(2)/sqrt((int(qb(2)*qb(2),x,[-1,1])));
% qb(3)=a(3)-int(q(1)*a(3),x,[-1,1])*q(1)-int(q(2)*a(3),x,[-1,1])*q(2);
% q(3)=qb(3)/sqrt(int(qb(3)*qb(3),x,[-1,1]));
% qb(4)=a(4)-int(q(1)*a(4),x,[-1,1])*q(1)-int(q(2)*a(4),x,[-1,1])*q(2)-int(q(3)*a(4),x,[-1,1])*q(3);
% q(4)=qb(4)/sqrt(int(qb(4)*qb(4),x,[-1,1]));
qb
q
% int(q(1)*q(3),x,[-1,1])
% int(q(1)*q(2),x,[-1,1])
% int(q(2)*q(3),x,[-1,1])
% int(q(2)*q(3)*q(1),x,[-1,1])
%scale:
y=2*x-3
q(1)=q(1)*sqrt(2)
q(2)=sqrt(2)*sqrt(3/2)*(2*x-3)
q(3)=sqrt(2)*sqrt(5/8)*(3*(2*x-3)^2-1)
c(1)=int(exp(x)*q(1),x,[1,2])
c(2)=int(exp(x)*q(2),x,[1,2])
c(3)=int(exp(x)*q(3),x,[1,2])
p=c*q'
plotx=1:pi/100:2;
ploty1=exp(plotx);
plot(plotx,ploty1);
ploty1=subs(p,'x',plotx);
hold on;plot(plotx,ploty1);
legend('f(x)','p(x)')