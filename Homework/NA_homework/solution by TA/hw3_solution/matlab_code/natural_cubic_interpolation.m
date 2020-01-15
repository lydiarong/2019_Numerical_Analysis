function [y] = natural_cubic_interpolation(datax,datay,x)
%NATURAL_CUBIC_POLINORMIAL : Function to calculate the 
%   natrual cubic polinormial at the datax
%   [y] = natural_cubic_interpolation(datax,datay,x)
%   access the parameter datax and datay to traning the cubic polinormial

[a b c d] = natural_cubic_spline(datax, datay);

y = ones(1,size(x(:),1));
for i = 1:size(datax(:),1)-1
    index = x >= datax(i) & x< datax(i+1);
    y(index) = a(i)*y(index) + b(i)*(x(index) - datax(i)) +...
        c(i)*(x(index) - datax(i)).^2 + d(i)*(x(index) - datax(i)).^3;
end

if x(end) == datax(end)
	y(end) = datay(end);
end

end
