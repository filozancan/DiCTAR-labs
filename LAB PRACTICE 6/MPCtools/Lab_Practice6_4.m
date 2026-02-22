clc, close all, clear all, format compact,
%% Problem 4 - Closed loop constrained QOC trough Receding Horizon Algorithm, input and state constraint
A=[0.9616 0.1878;-0.3756 0.8677]; B=[0.0192;0.1878]; x0=[-0.3;0.4]; Ts=0.2;

Q=diag([5,1]); %S=Q;
R=0.1;
nx=2; nu=1;
Hp=3;

% Matrices
A_cal= zeros(Hp*nx,nx);
for i=1:Hp
    A_cal( (i-1)*nx+1:i*nx, :)=A^i;
end

B_cal= zeros(Hp*nx,nu*Hp);
for i=1:Hp
    for j=1:i
        B_cal((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu)=A^(i-j)*B;
    end
end

Q_cal=kron(eye(Hp),Q);
R_cal=kron(eye(Hp),R);

% cost fuction
H=2*(B_cal'*Q_cal*B_cal+R_cal);
H=(H+H')/2;
F=2*A_cal'*Q_cal*B_cal;

% constrains
%input
u_sat=0.5;
Gu=[eye(Hp);-eye(Hp)];
hu=u_sat*ones(2*Hp,1);

%state
x_max=[0;0.5];
Gx=B_cal;
hx=-A_cal*x0+repmat(x_max,Hp,1);


% RH
sim_steps=40;
x_k=x0;

G=[Gu;Gx];
h=[hu;hx];

for kk=1:sim_steps
    %updating state constraint
    hx=-A_cal*x0+repmat(x_max,Hp,1);
    h=[hu;hx];
    
    % core logic
    U=quadprog(H,x_k'*F,G,h);
    x_traj(:,1)=A*x_k+B*U(1);
    u_traj(:,kk)=U(1);
    x_k=x_traj(:,kk+1);

end

figure(1)
stairs([0:1:sim_steps]*Ts, x_traj(1,:)), hold on, zoom on,
xlabel('t(s)'), ylabel('x_1(t)')

figure(2)
stairs([0:1:sim_steps]*Ts, x_traj(2,:)), hold on, zoom on,
xlabel('t(s)'), ylabel('x_2(t)')

figure(3)
stairs([0:1:sim_steps]*Ts, sqrt(x_traj(1,:).^2+x_traj(2,:).^2)), hold on, zoom on,
xlabel('t(s)'), ylabel('||x||_2(t)')

figure(4)
stairs([0:1:sim_steps-1]*Ts, u_traj), hold on, zoom on,
xlabel('t(s)'), ylabel('u(t)')


