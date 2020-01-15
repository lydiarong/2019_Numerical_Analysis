function F=roo(p)
syms x s z
s=[exp(-x) x*exp(-x) x^2*exp(-x) x^3*exp(-x)]
z=[x^0 x^1 x^2 x^3]
F(1)=double(subs(int(subs(z(1),x,p(1))*p(2)+subs(z(1),x,p(3))*p(4)-int(s(1),x,-1,1),x,-1,1)))   
F(2)=double(subs(int(subs(z(2),x,p(1))*p(2)+subs(z(2),x,p(3))*p(4)-int(s(2),x,-1,1),x,-1,1)))
F(3)=double(subs(int(subs(z(3),x,p(1))*p(2)+subs(z(3),x,p(3))*p(4)-int(s(3),x,-1,1),x,-1,1)))
F(4)=double(subs(int(subs(z(4),x,p(1))*p(2)+subs(z(4),x,p(3))*p(4)-int(s(4),x,-1,1),x,-1,1)))