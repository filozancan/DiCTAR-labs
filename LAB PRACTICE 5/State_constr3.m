clc,clear all,close all,format compact
%% STATE CONSTRAINT - increase of Hp=10
A=[1 0.0488;0 0.9512];
B=[0.0012;0.0488];
x0=[-0.8;0];
Ts=0.05;

Q=diag([100,1]);
R=1;


u_sat=0.6;

%PARAMETERS
N = 10;                         % Prediction horizon
nx = 2;                         % Dimensione stato
nu = 1;                         % Dimensione ingresso

% COSTRUZIONE DI A_cal
% A_cal = [A; A^2; ...; A^N]
A_cal = zeros(nx*N, nx);
for i = 1:N
    A_cal((i-1)*nx+1 : i*nx, :) = A^i;
end

% COSTRUZIONE DI B_cal
% Struttura Toeplitz:
% [B       0       ...        0
%  A*B     B       ...        0
%  ...
%  A^(N-1)*B  A^(N-2)*B ...    B ]

B_cal = zeros(nx*N, N*nu);
for i = 1:N
    for j = 1:i
        B_cal((i-1)*nx+1 : i*nx, (j-1)*nu+1 : j*nu) = A^(i-j) * B;
    end
end

% COSTRUZIONE DI Q_cal E R_cal (block diagonal)
% Q_cal = blkdiag(Q, ..., Q)  N volte
% R_cal = blkdiag(R, ..., R)  N volte

Q_cal = kron(eye(N), Q);
R_cal = kron(eye(N), R);

%%CALCOLO DI H E F PER IL PROBLEMA QUADRATICO

% H = 2*(B_cal' Q_cal B_cal + R_cal)
H = 2 * (B_cal' * Q_cal * B_cal + R_cal);
H = (H + H')/2;    % Simmetrizzazione numerica

% F = 2*A_cal' Q_cal B_cal
F = 2 * (A_cal' * Q_cal * B_cal);

%COSTRUZIONE VINCOLI
x_max = [0.06; 0.35];

% Gx = vincoli sullo stato
Gx = B_cal;
hx = -A_cal*x0 + repmat(x_max, N, 1);

% Gu e hu = vincoli sull’ingresso
Gu = [eye(N); -eye(N)];
hu = u_sat * ones(2*N, 1);

% G, h complessivi
G = [Gu; Gx];
h = [hu; hx];

% LOOP RECEDING HORIZON (MPC)
x_traj(:,1) = x0;
x_k = x0;

for kk = 1:500

    % Aggiornamento hx
    hx = -A_cal*x_k + repmat(x_max, N, 1);
    h = [hu; hx];

    % Risoluzione QP
    U = quadprog(H, x_k'*F, G, h);

    % Stato successivo applicando solo il primo ingresso
    x_traj(:,kk+1) = A*x_k + B*U(1);
    u_traj(kk) = U(1);

    x_k = x_traj(:,kk+1);
end

%PLOT RISULTATI
figure(1)
subplot(311)
stairs([0:500]*Ts, x_traj(1,:)), grid on
ylabel('x_1')
xlabel('t (s)')

subplot(312)
stairs([0:500]*Ts, x_traj(2,:)), grid on
ylabel('x_2')
xlabel('t (s)')

subplot(313)
stairs([0:500]*Ts, sqrt(x_traj(1,:).^2 + x_traj(2,:).^2)), grid on
yline(1e-4, ':r')
yline(-1e-4, ':r')
ylabel('||x||_2')
xlabel('t (s)')

figure(2)
stairs([0:499]*Ts, u_traj), grid on
ylabel('u')
xlabel('t (s)')