close all;clear all; clc; 
%% use Gram-Schmidt algorithm to construct orthogonal polynomials 
% p1, p2, p3, based on the basic 1, x, x^2
syms x
q1_bar = 1;
q1_bar_norm = int(q1_bar*q1_bar*exp(-x^2/2),-1,1);
q1 = q1_bar/sqrt(q1_bar_norm);

q2_bar = x - int(q1*x*exp(-x^2/2),-1, 1)*q1;
q2_bar_norm = int(q2_bar*q2_bar*exp(-x^2/2),-1,1);
q2 = q2_bar/sqrt(q2_bar_norm);

q3_bar = x*x - int(q1*x*x*exp(-x^2/2),-1, 1)*q1 - int(q2*x*x*exp(-x^2/2),-1, 1)*q2;
q3_bar_norm = int(q3_bar*q3_bar*exp(-x^2/2),-1,1);
q3 = q3_bar/sqrt(q3_bar_norm);
q1 
q2 
q3
%% check the result
double(int(q1*q2*exp(-x^2/2),-1, 1))
double(int(q1*q3*exp(-x^2/2),-1, 1))
double(int(q2*q3*exp(-x^2/2),-1, 1))