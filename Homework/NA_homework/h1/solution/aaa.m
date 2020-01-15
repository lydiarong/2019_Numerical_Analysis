function I = test()
for n = 1:15
    x(1,n) = 1e-16*10.^(n/1);
    I(1,n) = abs((exp(x)-exp(-x))/(2*x)- 1.0);
end
loglog(x,I)
end
