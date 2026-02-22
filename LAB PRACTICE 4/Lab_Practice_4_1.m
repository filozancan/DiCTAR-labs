clc, close all, clear all
%% Problem 1 - ZERO REGULATION PROBLEM

Ts=0.05;
A=[0 1;0 -1];
B=[0;1];
C=[1 0]; D=0;
x0=[0.8;0];

sys=ss(A,B,C,D);

% Discretization
sys_dt=c2d(sys,Ts, 'zoh');

Ad=sys_dt.a;
Bd=sys_dt.b;
Cd=sys_dt.c;
Dd=sys_dt.d;

% Reachability
Mr=ctrb(Ad, Bd);
rho_Mr = rank(Mr)

Q=diag([180,1]);
R=1;

% Observability
Cq=chol(Q);
Mo=obsv(Ad, Cq);

rho_Mo = rank(Mo);

K = dlqr(Ad, Bd, Q,R)

%% simulink
sys_x=ss(A,B,eye(2),0);
t_sim=15;

out=sim("Lab4_1simulink.slx")

figure(1)
plot(out.x.time, out.x.data(:,1), 'b', 'LineWidth', 1.45)
hold on, grid on, zoom on,
xlabel('t'), ylabel('x_1')

figure(2)
plot(out.x.time, out.x.data(:,2), 'b', 'LineWidth', 1.45)
hold on, grid on, zoom on,
xlabel('t'), ylabel('x_2')

figure(3)
plot(out.x.time,sqrt( out.x.data(:,1).^2 + out.x.data(:,2).^2), 'b', 'LineWidth', 1.45)
hold on, grid on, zoom on,
yline(1e-5, ':r', 'LineWidth', 1.45), yline(-1e-5, ':r', 'LineWidth', 1.45)
xline(5*1.02, ':g', 'LineWidth', 1.45), xline(5*0.98, ':g', 'LineWidth', 1.45)
xlabel('t'), ylabel('||x||_2')

figure(4)
plot(out.u.time, out.u.data, 'b', 'LineWidth', 1.45)
hold on, grid on, zoom on,
xlabel('t'), ylabel('u')
