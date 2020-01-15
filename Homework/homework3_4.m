clear all;
x0=1;
xn=2;
N=4;
h=(xn-x0)/N;
ind=x0:h:xn;
y=(ind).^-2;
z=1;
syms x;
t=0:N;
for i=1:size(y,2)
    for j=1:size(y,2)
        if j~=i
            z=z.*((x-t(j))/(t(i)-t(j)));        
        end   
    end 
    a(i)=int(z,x,[0,N]);
    z=1;
end
fprintf('%f\n',h*(a*y'));
int(1/(x*x),x,[1,2])
% (y(1)+4*y(2)+y(3))*h/3