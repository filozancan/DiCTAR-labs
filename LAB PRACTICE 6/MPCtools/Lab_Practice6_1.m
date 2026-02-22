clc, close all, clear all, format compact
%% Problem 1 - MPC design for output set-point tracking

A = [0 1;0 -1]; B = [0;1]; C = [1 0]; D=0; x0=[0;0];
Ts=0.05;
nx=2;
sys=ss(A,B,C,D);
%discretiation
sys_dt=c2d(sys,Ts, 'zoh');
Ad=sys_dt.a;
Bd=sys_dt.b;
Cd=sys_dt.c;
Dd=sys_dt.d;

% for simulink
sys_x=ss(A,B,eye(nx),D);

% design parameters for MPCtools
Cy=eye(nx); 
Cz=Cd;
Dz=0;
Cc=Cd;
Dc=0;

Q=1000; %Q1=1, Q2=100, Q3=1000 -> the performance don't get affected 
R=1;
cmode=0;
h=Ts;

% since tuning Q doesn't work I impoe an output constraint
z_max=[1]; % performance achieved!
z_min=[-inf];
u_max=[1];
u_min=[-1];
du_max=[inf];
du_min=[-inf];

% 2/0.05=40
Hp=20; %40 ->35 ->30 ->25 ->20 | 15 too low
Hu=15; % best value, at 10 is a liiitle bit slower
Hw=1;
z_blk=1;
u_blk=1;

% compute MPC controller
md=MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,z_blk,Hu,u_blk,du_max,du_min,u_max,u_min,z_max,z_min,Q,R,[],[],h,cmode,'qp_as');

%% SIMULATION
t_sim=4;
rho=1;
out=sim("Lab_Practice6_1_SML.slx");


figure(1)
plot(out.y.time, out.y.data, 'b','LineWidth',1.45)
hold on, grid on, zoom on,
% time constraint
xline(2*1.05, ':r')
xline(2*0.95, ':r')
% y constraint, since the input is a step=1 we expect y=1
yline(1.01, ':g','LineWidth',1.45)
yline(0.99, ':g','LineWidth',1.45)
xlabel('t(s)'), ylabel('y(t)')

% figure(2)
% plot(out.u.time, out.u.data, 'b','LineWidth',1.45)
% hold on, grid on, zoom on,
% xlabel('t(s)'), ylabel('u(t)')

% subplot(221)
% plot(out.x.time, out.x.data(:,1), 'b', 'LineWidth', 1.45)
% hold on, grid on, zoom on,
% xlabel('t(s)'), ylabel('x_1(t)')
% 
% subplot(222)
% plot(out.x.time, out.x.data(:,2), 'b', 'LineWidth', 1.45)
% hold on, grid on, zoom on,
% xlabel('t(s)'), ylabel('x_2(t)')
% 
% subplot(223)
% plot(out.x.time, sqrt(out.x.data(:,1).^2 + out.x.data(:,2).^2), 'b', 'LineWidth', 1.45)
% hold on, grid on, zoom on,
% xlabel('t(s)'), ylabel('||x||_2(t)')
