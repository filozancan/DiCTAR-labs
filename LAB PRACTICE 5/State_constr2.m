clc,clear all,close all,format compact
%% STATE CONSTRAINT - no solution due to low Prediction horizon (Hp=3)
A=[1 0.0488;0 0.9512];
B=[0.0012;0.0488];
x0=[-0.8;0];
Ts=0.05;

Q=diag([100,1]);
R=1;

%Compute calligraphic matrices 
A_cal=[A;A^2;A^3];
B_cal=[[B zeros(2,1) zeros(2,1)]; [A*B B zeros(2,1)];[A^2*B A*B B]];
Q_cal=blkdiag(Q,Q,Q);   
R_cal=blkdiag(R,R,R);

%Compute the Hessian of the quadratic form   
H=2*(B_cal'*Q_cal*B_cal+R_cal);
H=(H+H')/2;
F=2*A_cal'*Q_cal*B_cal;
u_sat=0.6;

%Modifichiamo x_max mettendo 0.01 alla prima componente, in seguito a 0
%Define state constraint, prima x_max=[0.06; 0.35]
x_max=[0.01; 0.35]
Gx=B_cal;
hx=-A_cal*x0+repmat(x_max,3,1);   %given a matrix and replicate it according to a matrix with some dimension

Gu=[eye(3); -eye(3)];
hu=u_sat*ones(6,1);
G=[Gu;Gx];
x_traj(:,1)=x0;
x_k=x0;

%RH procedure
for kk=1:500
    hx=-A_cal*x_k+repmat(x_max,3,1);
    h=[hu;hx];
    U=quadprog(H,x_k'*F,G,h);
    x_traj(:,kk+1)=A*x_k+B*U(1);
    %Compute one step ahead prediction using the first component of the
    %optimizer
    u_traj(kk)=U(1);
    x_k=x_traj(:,kk+1); %initialize x_k for mext step
end

figure(1)
subplot(311)
stairs([0:1:500]*Ts,x_traj(1,:)), grid on, hold on
ylabel('x_1')
xlabel('t(s)')
subplot(312)
stairs([0:1:500]*Ts,x_traj(2,:)), grid on, hold on
ylabel('x_2')
xlabel('t(s)')
subplot(313)
stairs([0:1:500]*Ts,sqrt(x_traj(1,:).^2+x_traj(2,:).^2)), grid on, hold on
yline(1e-4,':r')
yline(-1e-4,':r')
ylabel('||x||_2')
xlabel('t(s)')

figure(2)
stairs([0:1:499]*Ts,u_traj), grid on, hold on
ylabel('u')
xlabel('t(s)')
return
%We need an input in the opposite direction (and it's the optimum input!)
%Abbiamo un picco molto negativo!
%Tuttavia u_sat cuts the input a -0.6 --->ho bisogno di ridurre l'input
%Questo accade solo se considero state constraints, perchè l'area
%contsraints non è un quadrato ma un poligono di linee in una zona
%specifica che non ci permette di trovare una soluzione al problema
%Ma dobbiamo garantire che il metodo di ottimizzazione fornisca una soluzione

% With a short prediction horizon (Hp=3), the controller cannot "see" far enough into the future
% to slow down the system in time to meet the strict constraint