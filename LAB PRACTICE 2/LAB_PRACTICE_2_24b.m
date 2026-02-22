clc, clear all, close all, format compact,
%% LAB PRACTICE 2
Ts = 0.01;
s = tf('s');
Gcont = 160/(s^2+36*s+400);

% discretization
G=zpk(c2d(Gcont,Ts,'zoh'))
% pzmap(G, 'r')
% hold on, zoom on, axis('equal')

[z,p,k]=zpkdata(G,'v');
B=k*[1 -z];
A=conv([1 -p(1)],[1 -p(2)]);

A_plus=A;
A_min=1;
B_plus=1;
B_min=B;

% transient requirements
ts1=1; s_hat=0.19;

zeta=abs(log(s_hat))/(sqrt(pi^2+ log(s_hat)^2));
wn = 4.6/(ts1*zeta);

% 3 poles
p1c=-zeta*wn+j*wn*sqrt(1-zeta^2);
p2c=-zeta*wn-j*wn*sqrt(1-zeta^2);
p3c=-5*zeta*wn;

p1d=exp(Ts*p1c);
p2d=exp(Ts*p2c);
p3d=exp(Ts*p3c);

Am=poly([p1d, p2d, p3d]);

%solution settings
A_dioph=conv([1 -1], A_min);
B_dioph= B_min;

G1=dcgain(G);
Ap1=polyval(A_plus,1);
yd2=0.1;
k=Ts/(G1*Ap1);


M_S=[[A_dioph(:);0;0],[0;A_dioph(:);0],[0;0;A_dioph(:)],[0;B_dioph(:);0],[0;0;B_dioph(:)];k,k,k,-yd2,-yd2]

Gamma=[Am';0];
Theta=M_S\Gamma;

R1=Theta(1:3)';
S1=Theta(4:5)';

S=conv(A_plus,S1);
R=conv([1 -1], R1);

C=minreal(zpk(tf(S,R,Ts)))
G=minreal(G)

%% SIMULINK

rho=1;
d2=0;
t_sim=15;

out=sim("simulink_lab2_24b.slx")

%transient
figure(1)
plot(out.y.time, out.y.data,'b', 'LineWidth',1.45)
hold on, zoom on, grid on,
yline(1.01, ':r')
yline(0.99, ':r')
xlabel('t(s)'), ylabel('y(t)')

%1a
rho=0;
d2=1;
out=sim("simulink_lab2_24b.slx")
figure(2)
plot(out.y.time, out.y.data,'b', 'LineWidth',1.45)
hold on, zoom on, grid on,
xlabel('t(s)'), ylabel('y_d2(t)')

%1b
rho=3;
d2=0;
out=sim("simulink_lab2_24b.slx")
figure(3)
plot(out.e.time, out.e.data,'b', 'LineWidth',1.45)
hold on, zoom on, grid on,
xlabel('t(s)'), ylabel('e_r(t)')