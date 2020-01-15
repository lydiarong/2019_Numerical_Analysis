x = linspace(0,1,11);
fx = x.*x;

[a b c d] = natural_cubic_spline(x,fx);

h = x(2) - x(1);
intergrate = sum(4*h*c.*c + 12*h*h*c.*d + 12*h*h*h*d.*d);
fprintf('%f\n',intergrate);
