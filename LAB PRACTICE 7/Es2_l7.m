clc
clear all
close all

s=tf('s')
Gcont=40/(s^2+4*s-10)
Ts=0.005
G=zpk(c2d(Gcont,Ts,'zoh'))
[z_G,p_G,k_G]=zpkdata(G,'v')

G_1=dcgain(G)

%Define A and B
B=k_G*[1 -z_G]
A=conv([1 -p_G(1)],[1 -p_G(2)])

%Define requirements
s_hat=0.25
t_s1=2.5

zeta=abs(log(s_hat))/(sqrt(pi^2+(log(s_hat))^2))
wn=4.6/(zeta*t_s1)

%Cardiod region
% figure(1)
% pzplot(G,'r')
% hold on
% zgrid(zeta,0,Ts)
% axis('equal')

%Define A+ A- B+ B-
A_plus=[1 -p_G(2)]
A_minus=[1 -p_G(1)]
B_plus=1
B_minus=B

%Define eigenvalues in CT
p1c=-zeta*wn+j*wn*sqrt(1-zeta^2)
p2c=-zeta*wn-j*wn*sqrt(1-zeta^2)
p3c=-zeta*wn*30
p4c=p3c

%Define eigenvalues in DT
p1d=exp(p1c*Ts)
p2d=exp(p2c*Ts)
p3d=exp(p3c*Ts)
p4d=p3d

A_m=poly([p1d, p2d, p3d, p4d])

%Define A_dioph and B_dioph
A_dioph=conv([1 -1],[1 -p_G(1)])
B_dioph=[B]

%Additional condition
y_d1=-0.25
A_plus_1=polyval(A_plus,1)
%return
k=-A_plus_1

%Silvester matrix
M_S=[[[A_dioph(:);0;0],[0;A_dioph(:);0],[0;0;A_dioph(:)],[0;B_dioph(:);0;0],[0;0;B_dioph(:);0],[0;0;0;B_dioph(:)]];[y_d1 y_d1 y_d1 k k k]]

%Solve the system
Gamma=[A_m';0]
theta=M_S\Gamma

R1=theta(1:3)'
S1=theta(4:6)'
R=conv([1 -1],R1)
S=conv(A_plus,S1)
C=zpk(tf(S,R,Ts))
G=zpk(G)

%Simulation
rho=1;
d1=0;
d2=0;
t_sim=5;
out=sim('Es2_l7_simulink')

figure(2)     %Metti rho=1
hold on
plot(out.r.time,out.r.data,'k','linew',1.2)
plot(out.y.time,out.y.data,'b','linew',1.2)
yline(1.01,':r','linew',1.5)
yline(0.99,':r','linew',1.5)
zoom on, grid on
xlabel('t(s)'),ylabel('y(t)')

figure(3)   
plot(out.e.time,out.e.data,'b','linew',1.2)
zoom on, grid on
xlabel('t(s)'),ylabel('e(t)')

% figure(4)   %Metti d1=1
% plot(out.y.time,out.y.data,'b','linew',1.2)
% zoom on, grid on
% xlabel('t(s)'),ylabel('y_d1(t)')

% figure(5)   %Metti d2=1
% plot(out.y.time,out.y.data,'b','linew',1.2)
% zoom on, grid on
% xlabel('t(s)'),ylabel('y_d2(t)')

