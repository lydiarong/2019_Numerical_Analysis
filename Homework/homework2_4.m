% function p = homework2_4(x0,xn,N)
clear all;close all;
% if nargin ==0
    x0=0;
    xn=1;
    N=10;
% end
h=(xn-x0)/N;
ind=x0:h:xn;
y=ind.*ind;
D=zeros(N-1);

for i = 1:N-1
    D(i,i) = 4*h;  
    if (i-1)>0
    D(i,i-1) = h;
    end
    if (i+1)<N
    D(i,i+1) =h;
    end
end
R=zeros(N-1,1);
for i = 2:N
    R(i-1)=(3/h)*(y(i+1)-2*y(i)+y(i-1));
end
c = D\R;
c=[0;c;0]'
a=y(:,1:N)
d=(c(2:N+1)-c(1:N))/(3*h)
b=((y(2:N+1)-y(1:N))/h)-((2*c(1:N)+c(2:N+1))*(h/3))
syms x;
for i=1:N
    p{i}=a(i)+b(i)*(x-ind(i))+c(i)*(x-ind(i))^2+d(i)*(x-ind(i))^3;
end
%-c(1:N)*h-d(1:N)*h*h
%+((c(1:N)-4*c(2:N+1))*(h/3))
%-(c(1:N)/(3))*2*h-c(2:N+1)/(3)*h
plotx=[x0:pi/100:xn,xn];
ploty1=plotx.^2;
plot(plotx,ploty1);
for i=1:N-1
plotx=x0+h*(i-1):pi/100:x0+h*i;
ploty1=subs(p{i},'x',plotx);
hold on;plot(plotx,ploty1);
end
hold on;plot(ind,y,'ro');
legend('f(x)','p1(x)','p2(x)','p3(x)','p4(x)','p5(x)','p6(x)','p7(x)','p8(x)','p9(x)','data');
intergrate = sum(4*h*c(1:N).*c(1:N) + 12*h*h*c(1:N).*d + 12*h*h*h*d.*d);
fprintf('%f\n',intergrate);