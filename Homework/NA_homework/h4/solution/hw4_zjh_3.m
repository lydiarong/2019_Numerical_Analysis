clc;clear all;close all;
syms x;
f=diff(exp(-x.^2),x,4);
%the fourth derivative of exp(-x^2)
f4=f
%get the maxmium 
fmax=max(subs(f,x,[0:1]))
%the numerical integration error bound
error=fmax/2880