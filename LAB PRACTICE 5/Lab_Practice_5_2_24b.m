clc, close all, clear all, format compact,
%% PROBLEM 2 - constrained finite horizon quadratic optimal control problem
A = [0.3 1.5;0.5 -0.4]; B=[0;1]; 
x_k=[20;20]; %x0
Ts=0.1;
Hp=3; % prediction horizon
Q=diag([20,5]); S=Q; R=5;

% build matrices
A_cal=[A;A^2;A^3]; B_cal=[[B zeros(2,1) zeros(2,1)];[A*B B zeros(2,1)];[A^2*B A*B B]];
Q_cal=blkdiag(Q,Q,Q); R_cal=blkdiag(R,R,R);

% preliminary steps
H=2*(B_cal'*Q_cal*B_cal+R_cal);
H=(H+H')/2;
F=2*A_cal'*Q_cal*B_cal;

%% constraint on input - saturation
u_sat=13;
G=[eye(3);-eye(3)]; 
h=u_sat*ones(6,1);

% Optimal solution

U=quadprog(H,F'*x_k,G,h)

