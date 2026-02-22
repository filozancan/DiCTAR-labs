clc, close all, clear all, format compact,
%% Problem 2 - 1dof control architecture

% 1 - stability
Ts=1;
z=tf('z',Ts);
C=0.01*(z-0.2)/(z-0.8);
G=(z-1)/(z^2-0.2*z+0.02);

L=series(C,G);
H=minreal(zpk(tf(L,1+L,Ts)))

pole(H) % since all the poles have magnitude <1, the system is stable

% 2 - response computation
r=2*z/(z-1);
W=H;

y=W*r;
[num,den]=tfdata(y,'v');
[ry,py,ky]=residuez(num,den)
% complex conjugates ->cosine
nu=abs(py(3))
theta=angle(py(3))
M=abs(ry(3)), M2=2*M
phi=angle(ry(3))

% 3 - final value. Since G has a zero in 1, y->0 when r=2*epsilon(k)
% 3 - final value. Since G has a zero in 1, y->0 when d1=epsilon(k)

