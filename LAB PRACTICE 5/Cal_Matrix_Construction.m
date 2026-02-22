clc,clear all,close all,format compact

A=[1 0.0488;0 0.9512];
B=[0.0012;0.0488];
x0=[-0.8;0];
Ts=0.05;

Q=diag([100,1]);
R=1;


u_sat=0.6;

%PARAMETERS
N = 5;                         % Prediction horizon
nx = 2;                         % Dimensione stato
nu = 1;

% COSTRUZIONE DI A_cal
% A_cal = [A; A^2; ...; A^N]

A_cal=zeros(N*nx,nx);
for i=1:N
    A_cal((i-1)*nx+1 : i*nx , : )=A^i; % starting from row [(i-1)*nx+1] to [i*nx] place A^i into matrix
end

A_cal

% COSTRUZIONE DI B_cal
% Struttura Toeplitz:
% [B       0       ...        0
%  A*B     B       ...        0
%  ...
%  A^(N-1)*B  A^(N-2)*B ...    B ]

B_cal=zeros(N*nx,N*nu);
for i=1:N
    for j=1:i
        B_cal( (i-1)*nx+1 : i*nx , (j-1)*nu+1 : j*nu )=A^(i-j)*B;
    end
end

B_cal

% COSTRUZIONE DI Q_cal E R_cal (block diagonal)
% Q_cal = blkdiag(Q, ..., Q)  N volte
% R_cal = blkdiag(R, ..., R)  N volte

Q_cal = kron(eye(N), Q); % moltiplica I per Q o R 
R_cal = kron(eye(N), R);
