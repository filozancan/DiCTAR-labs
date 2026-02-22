clc, close all, clear all
%% Problem 2

Ts=0.005;
s_hat=0.25; ts1=2.5;
zeta=abs(log(s_hat))/sqrt(pi^2+ log(s_hat)^2);
wn=4.6/(zeta*ts1);
s=tf('s');
Gcont=40/(s^2+4*s-10);

G=zpk(c2d(Gcont,Ts,'zoh'))
[z,p,k]=zpkdata(G,'v')
B=k*[1 -z];
A=conv([1 -p(1)], [1 -p(2)]);
% pzmap(G), hold on, axis("equal")
Bmin=B; Bplus=1;
Aplus=[1 -p(2)]; Amin=[1 -p(1)];

p1c=-zeta*wn+j*wn*sqrt(1-zeta^2);
p2c=-zeta*wn-j*wn*sqrt(1-zeta^2);
p3c=-5*zeta*wn;
p4c=p3c;

p1d=exp(Ts*p1c);
p2d=exp(Ts*p2c);
p3d=exp(Ts*p3c);
p4d=p3d;

Am=poly([p1d, p2d,p3d, p4d]);

Adiop=conv([1 -1], Amin)
Bdiop=Bmin
Ap1=polyval(Aplus,1); G1=dcgain(G);
x=0.1*Ts/(Ap1*0.0005*G1)

M_S=[ [Adiop(:);0;0], [0;Adiop(:);0], [0;0;Adiop(:)], [0;Bdiop(:);0;0], [0;0;Bdiop(:);0], [0;0;0;Bdiop(:)]; x,x,x,-1,-1,-1 ]

Gamma=[Am';0];
Theta=M_S\Gamma;

R1=Theta(1:3)';
S1=Theta(4:end)';

S=conv(S1,Aplus);
R=conv([1 -1], R1);

C=minreal(zpk(tf(S,R,Ts)))
G=minreal(G);

