%minf(x) s.t g(x)=0
%f=f=x1^2+x2^2;
%g=x1+x2-1
%
function newton()
    clc;
    dimension=2;
    syms x1 x2 delta_x1 delta_x2 lamda
    f=x1^2+x2^2;
    g=x1+x2-1;
    A=[1 1];
    b=1;
    v=[x1;x2];
    delta_v=[delta_x1;delta_x2];   
    x_ini=[0.1;0.1];
    new_lamda=0.1
    new_delta_v=[0.1;0.1]
    
    J=jacobian(f,v)
    H=jacobian(J,v)   
    cond=abs(J*delta_v)+abs(lamda*(A*v-b))
    Z1=[H,A';A,0]
    B1=[-J';b-A*v]
    Z2=inv(Z1)*B1
    KKT=subs(Z2,v,x_ini)

    while subs(subs(cond,lamda,new_lamda),[v,delta_v],[x_ini,new_delta_v])>0.0001
       double(subs(subs(cond,lamda,new_lamda),[v,delta_v],[x_ini,new_delta_v]));   
       double(new_lamda);
       double(new_delta_v);
       KKT=subs(Z2,v,x_ini);
       new_delta_v=KKT(1:2);
       x_ini=x_ini+0.1*new_delta_v;
       double(subs(g,v,x_ini));
       new_lamda=KKT(3);
    end   
disp 'the result is:'
double(x_ini)