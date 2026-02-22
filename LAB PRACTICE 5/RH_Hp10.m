clc,clear all,close all,format compact
%% CONSTRAINTS - increase of Hp=10
A=[1 0.0488;0 0.9512];
B=[0.0012;0.0488];
x0=[-0.8;0];
Ts=0.05;

Q=diag([100,1]);
R=1;
x_k=x0;

%constraints
u_sat=0.6;
x_max=[0.06; 0.35];

%PARAMETERS
N = 10;                         % Prediction horizon
nx = 2;                         % Dimensione stato
nu = 1;                         % Dimensione ingresso

% 1 - MATRICES COMPUTATION

A_cal=zeros(nx*N,nx);
for i=1:N
    A_cal( (i-1)*nx+1:i*nx  , : )=A^i;
end

B_cal=zeros(N*nx,N*nu);
for i=1:N
    for j=1:i
        B_cal( (i-1)*nx+1:i*nx , (j-1)*nu+1:j*nu )=A^(i-j)*B;
    end
end

Q_cal=kron(eye(N),Q);
R_cal=kron(eye(N),R);

% 2 - COST FUNCTION 
H = 2*(B_cal'*Q_cal*B_cal+R_cal);
H=(H+H')/2;
F=2*A_cal'*Q_cal*B_cal;

% 3- CONSTRAINTS
% 3.1 - input
Gu=[eye(N);-eye(N)];
hu=u_sat*ones(2*N,1);

% 3.2 - state
Gx=B_cal;
hx=-A_cal*x_k+repmat(x_max,N,1);

% 4 - RECEDING HORIZON ALGORITHM
sim_step=500;
x_traj(:,1)=x_k;

G=[Gu;Gx];
h=[hu;hx];

for kk=1:sim_step
    
    % my constraint depends on current state, need to update at every
    % iteration
    hx=-A_cal*x_k+repmat(x_max,N,1);
    h=[hu;hx];

    % CORE LOGIC
    U=quadprog(H,x_k'*F,G,h);
    x_traj(:,kk+1)=A*x_k+B*U(1);
    u_traj(:,kk)=U(1);
    x_k=x_traj(:,kk+1);

end

figure(1)
stairs([0:1:sim_step]*Ts,x_traj(1,:)), grid on, zoom on, hold on
xlabel('t(s)'), ylabel('x_1(t)')

figure(2)
stairs([0:1:sim_step]*Ts,x_traj(2,:)), grid on, zoom on, hold on
xlabel('t(s)'), ylabel('x_2(t)')

figure(3)
stairs([0:1:sim_step]*Ts, sqrt(x_traj(1,:).^2+x_traj(2,:).^2)), grid on, zoom on, hold on
yline(1e-4,':r')
yline(-1e-4,':r')
xlabel('t(s)'), ylabel('||x||_2(t)')

figure(4)
stairs([0:1:sim_step-1]*Ts,u_traj), grid on, zoom on, hold on
xlabel('t(s)'), ylabel('u(t)')

