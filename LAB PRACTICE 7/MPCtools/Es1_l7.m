clc
clear all
close all

%Define the ct state matrices and initial condition
A=[0 1; 0 -1];
B=[0; 1];
C=[1 0];
D=0
sys=ss(A,B,C,D)
sys_x=ss(A,B,eye(2),0);  %for state feedback
x0=[1; 0];

%Discretize the model
Ts=0.05;
sys_dt=c2d(sys,Ts,'zoh');
Ad=sys_dt.a;
Bd=sys_dt.b;
Cd=sys_dt.c;
Dd=sys_dt.d;

%Set design parameters for MPCtools   (we have two controlled outputs)
Cy=eye(2);  %measured output -->all states measured
Cz=eye(2);   %Controlled output
Dz=[0;0];
Cc=eye(2);  %constrained output=controlled output
Dc=[0;0];

Q=diag([55 3])   %weight of the controlled output (same dimension of the controlled output that is a scalar)
R=1
cmode=0;  %state feedback
h=Ts;   %sample time
z_max=[inf inf];   %output constraints (we hve always define constraints, for the toolbox)
z_min=-[inf inf];
u_max=[5.5];   %input constraints
u_min=[-5.5];
du_max=[3];  %constraints on the input rates
du_min=[-3];
Hp=12;   %Prediction horizon    0.035 con Ts=0.001  mi servono 40
Hu=6;   %Control horizon
Hw=1;    %First penalty sample
z_blk=1;   %no blocking
u_blk=1;   %no blocking

md=MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,z_blk,Hu,u_blk,du_max,du_min,u_max,u_min,z_max,z_min,Q,R,[],[],h,cmode,'qp_as');
%The predictive controller has been designed

%We need to build a small simulink file
%1)Simulinl block that realize md
t_sim=5;
rho=1;
out=sim('Es1_l7_simulink.slx');
figure(1)
subplot(221)
plot(out.x.time,out.x.data(:,1),'b','linew',1.0)
hold on, grid on
ylabel('x_1(t)')
subplot(222)
plot(out.x.time,out.x.data(:,2),'b','linew',1.0)
hold on, grid on
ylabel('x_2(t)')
subplot(223)
plot(out.x.time,sqrt(out.x.data(:,1).^2+out.x.data(:,2).^2),'b','linew',1.0)
hold on, grid on
yline(1e-4,':r','linew',1.0)
yline(-1e-4,':r','linew',1.0)
xline(2.5*1.05,':g','linew',1.0)
xline(2.5*0.95,':g','linew',1.0)
ylabel('||x(t)||_2')
xlabel('t')
subplot(224)
plot(out.u.time,out.u.data,'b','linew',1.0)
hold on, grid on
ylabel('u(t)')
xlabel('t')

