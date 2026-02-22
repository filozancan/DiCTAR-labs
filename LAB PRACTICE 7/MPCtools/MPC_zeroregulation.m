clc, close all, clear all, format compact
%% - MPC example 2 ORIGINAL
% CT system, 2 states!
A = [0 1;900 0]; B = [0;-10]; C = [600 0]; D=0;

sys=ss(A,B,C,D);
sys_x=ss(A,B,eye(2),0); % for state feedback
x0=[-0.25;0];

% Discretization, MPS works in DT!
Ts=0.001;
sys_dt=c2d(sys,Ts, 'zoh');
Ad=sys_dt.a;
Bd=sys_dt.b;
Cd=sys_dt.c;
Dd=sys_dt.d;

% set design parameters for MPCTools
Cy=eye(2); % defines the output, alway I -> all states measured
Cz=eye(2); % controlled output-> state variable, I Cz=Cd
Dz=[0;0]; %!! 2 states 
Cc=eye(2); % constrained output=controlled output
Dc=[0;0];
Q=diag([1 1]); % same dimension of the controll output, so 2 %%q11->10, q11=1000
R=1;
cmode=0; % state feedback
h=Ts;

%output contraints, ALWAYS to be provided!
z_max= [inf inf];
z_min= [-inf -inf];

u_max= [30]; %input
u_min= [-30];

du_max= [inf]; %input rate, in this case no contraints
du_min= [-inf];

% Set design parameters for MPCtools cont
%Prediction
Hp=40; % prediction horizon Ts=1ms ->t_sampling is almost 40*Ts
Hu=40; % control horizon, Hu=Hp
Hw=1; % first penalty sample

z_blk=1; % no blocking
u_blk=1; % same

% compute MPC
md = MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,z_blk,Hu,u_blk,du_max,du_min,u_max,u_min,z_max,z_min,Q,R,[],[],h,cmode,'qp_as');

%% SIMULATION
t_sim = 0.6;
rho=1;
out=sim("MPC_example2.slx")

figure(1)
subplot(211)
plot(out.y.time, out.y.data, 'b', 'LineWidth',1.45)
yline(1e-4, ':r', 'LineWidth',1.45)
yline(-1e-4, ':r', 'LineWidth',1.45) 
hold on, grid on
ylabel('y(t)'),  xlabel('t(s)')

subplot(212)
plot(out.u.time, out.u.data, 'b', 'LineWidth',1.45)
hold on, grid on
ylabel('u(t)'), xlabel('t(s)')

%% 2 PROBLEMS - FEASIBILITY AND STABILITY