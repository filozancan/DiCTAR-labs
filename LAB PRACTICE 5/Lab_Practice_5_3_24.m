clc, close all, clear all, format compact,
%% PROBLEM 3 - constrained finite horizon quadratic optimal control problem with Receding Horizon
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

G=[eye(3);-eye(3)];
u_sat=13;
h=u_sat*ones(6,1);

%% Receding Horizon
t_sim=20;
x_traj(:,1)=x_k; % initialize trajectory vector with initial condition, the first column is the initial condition
% 2 states, 20+1 steps+initial condition -> 2x21 matrix


% x_k'*F 
for kk=1:t_sim
    U=quadprog(H,x_k'*F,G,h); % step 1 - solve QP
    x_traj(:,kk+1)=A*x_k+B*U(1); % compute step ahead prediction with 1st U component
    u_traj(kk)=U(1); % select only the first optimal input
    x_k=x_traj(:,kk+1); % updates state
end
% 1st state
figure(1)
stairs([0:1:t_sim]*Ts, x_traj(1,:)), grid on, zoom on, hold on %21 time steps (21x1), 21 states (1x21) 
xlabel('t(s)'), ylabel('x_1(t)')
% 2nd state
figure(2)
stairs([0:1:t_sim]*Ts, x_traj(2,:)), grid on, zoom on, hold on
xlabel('t(s)'), ylabel('x_2(t)')
%norm
figure(3)
stairs([0:1:t_sim]*Ts, sqrt(x_traj(1,:).^2+ x_traj(2,:).^2)), grid on, zoom on, hold on
yline(1e-4,':r', 'LineWidth',1.45)
yline(-1e-4,':r', 'LineWidth',1.45)
xlabel('t(s)'), ylabel('||x||_2(t)')
% input, Remember that u is computet up to Hp-1
figure(4)
stairs([0:1:t_sim-1]*Ts, u_traj),  grid on, zoom on, hold on
xlabel('t(s)'), ylabel('u(t)')