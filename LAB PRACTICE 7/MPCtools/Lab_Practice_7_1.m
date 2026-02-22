clc, close all, clear all,
%% Problem 1 - MPC zero-regulation

A=[0 1;0 -1]; B=[0;1]; C=[1 0]; D=0; x0=[1;0];
Ts=0.05; nx=2;

sys=ss(A,B,C,D); sys_x=ss(A,B,eye(nx),0);
sysd=c2d(sys, Ts, 'zoh');
Ad=sysd.a; Bd=sysd.b; Cd=sysd.c; Dd=sysd.d;

% set parameters

Cy=eye(nx);
Cz=eye(nx); Dz=[0;0];
Cc=eye(nx); Dc=Dz;

h=Ts; cmode=0;

z_max=[inf inf];
z_min=-[inf inf];

u_max=[5.5];
u_min=[-5.5];

du_max=[3];
du_min=[-3];

Q=diag([1000, 1]); R=1;
Hp=10;
Hc=10;
Hw=1;
z_blk=1; u_blk=1;

md=MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw, z_blk, Hc, u_blk, du_max, du_min, u_max, u_min, z_max, z_min, Q,R,[], [],h,cmode,'qp_as');

%% SIM

t_sim=3.5;
out=sim("Lab_Practice_7_1simulink.slx");

figure(1)
plot(out.x.time, out.x.data(:,1), 'k', 'LineWidth', 1.45)
hold on, grid on,
xlabel('t'), ylabel('x_1')

figure(2)
plot(out.x.time, out.x.data(:,2), 'k', 'LineWidth', 1.45)
hold on, grid on,
xlabel('t'), ylabel('x_2')

figure(3)
plot(out.x.time, sqrt(out.x.data(:,1).^2+  out.x.data(:,2).^2), 'k', 'LineWidth', 1.45)
hold on, grid on,
yline(1e-4, ':r', 'LineWidth', 1.3), yline(-1e-4, ':r', 'LineWidth', 1.3)
xline(2.5*1.05, ':g', 'LineWidth',1.3),xline(2.5*0.95, ':g', 'LineWidth',1.3),
xlabel('t'), ylabel('||x||_2')

figure(4)
plot(out.u.time, out.u.data, 'k', 'LineWidth', 1.45)
hold on, grid on,
xlabel('t'), ylabel('u')