function [a b c d] = natural_cubic_spline(x,y)
% NATURAL_CUBIC_SPLINE: Function to compute a natural cubic polynomial
%
% Assume a cubic polynomial has the form of
%
%  y_i = a_i + b_i*x + c_i*x*x + d_i*x*x*x
%
%  and with derivative of:
%
%  dy_i = b_i + 2*c_i*x + 3*d_i*x*x
%
% Solution is quite simple: Form a linear system of input values of y 
% sampled at points x (for y)
% 
% Form a MATRIX A for all value of x. Form a (column) vector b for
% corresaponding values of y
%
% And with the natural boundary conditions
% 
% d^2y_0 (x_0) = 2*c_0 = 0
% d^2y_{n-1} (x_n) = 2*c_{n-1} + 6*d_{n-1}*h_{n-1} = 0
%
% Use MATLAB \ operator to solve the system
%
% Inputs: x --- values of x where y samples are given
%         y --- values of y at given x values
%
% All inputs assumed row vectors.
% 
% Outputs: a,b,c,d --- parameters of natural cubic polynomial


% Get number of x data points
n = size(x(:),1);

% Construct the initial coefficient vector
a = zeros(n,1);
b = zeros(n,1);
c = zeros(n,1);
d = zeros(n,1);
h = zeros(n,1);

% Construct the difference metrix 
A = sparse(-diag(ones(n,1)) + diag(ones(n-1,1),1));
h = A*x(:)

% Construct the B of Dc = B
B = A*y(:);
B = B./h;
B = 3*(B - [0; B(1:end-1)])
B(1) = 0;B(end) = 0;

% Construct the D of Dc = B
D = sparse(zeros(n,n));
for i = 2:n-1
	D(i,i-1) = h(i-1);
	D(i,i) = 2*(h(i-1) + h(i));
	D(i,i+1) = h(i);
end
D(1,1) = 1;D(end,end) = 1;

% Solving Dc = B
c = D\B

% By the condiction y_i = a_i, we could get
a = y(:);

% Solve d from 3*di*hi = ci+1 - ci
d = A*c./h /3

% Solve b from bi*hi = ai+1 - ai -ci*hi*hi -di*hi*hi*hi
b = A*y(:)./h - c.*h - d.*h.*h
A*y(:)./h
c.*h
d.*h.*h
% output a,b,c,d
a = a(1:n-1);
b = b(1:n-1);
c = c(1:n-1);
d = d(1:n-1);
end
