clc, close all, clear all, format compact,
%% Problem 3 - Constrained QOC trough Receding Horizon Algorithm, input constraint
A=[0.9616 0.1878;-0.3756 0.8677]; B=[0.0192;0.1878]; x0=[-0.3;0.4]; Ts=0.2;

Q=diag([5,1]); %S=Q;
R=0.1;
nx=2; nu=1;
Hp=3;
% compute matrices

A_cal=zeros(nx*Hp,nx);
for i=1:Hp
    A_cal((i-1)*nx+1:i*nx , :)=A^i;
end

B_cal=zeros(nx*Hp,nu*Hp);
for i=1:Hp
    for j=1:i
        B_cal( (i-1)*nx+1:i*nx , (j-1)*nu+1:j*nu)=A^(i-j)*B;
    end
end

Q_cal=kron(eye(Hp),Q);
R_cal=kron(eye(Hp),R);

% input constraint
u_sat=0.5;

% cost fuction
H=2*(B_cal'*Q_cal*B_cal+R_cal);
H=(H+H')/2;
F=2*A_cal'*Q_cal*B_cal;

G=[eye(Hp);-eye(Hp)];
h=u_sat*ones(2*Hp,1);

% RH
sim_steps=40;
x_k=x0;
x_traj(:,1)=x_k;

for kk=1:sim_steps
    U=quadprog(H,x_k'*F,G,h);
    x_traj(:,kk+1)=A*x_k+B*U(1);
    u_traj(:,kk)=U(1);
    x_k=x_traj(:,kk+1);
end

figure(1)
stairs([0:1:sim_steps]*Ts, x_traj(1,:)), hold on, grid on, zoom on
xlabel('t(s)'), ylabel('x_1(t)')

figure(2)
stairs([0:1:sim_steps]*Ts, x_traj(2,:)), hold on, grid on, zoom on
xlabel('t(s)'), ylabel('x_2(t)')

sqrt(x_traj(1,:).^2+x_traj(2,:).^2)

figure(3)
stairs([0:1:sim_steps]*Ts, sqrt(x_traj(1,:).^2+x_traj(2,:).^2)), hold on, grid on, zoom on
yline(1e-4, ':r')
yline(-1e-4, ':r')
xlabel('t(s)'), ylabel('||x||_2(t)')


figure(4)
stairs([0:1:sim_steps-1]*Ts, u_traj), hold on, grid on, zoom on
xlabel('t(s)'), ylabel('u(t)')