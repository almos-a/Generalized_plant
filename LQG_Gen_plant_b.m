% Generalised plant without reference input 
clc
clear
%model
A = [0 0 1.132 0 -1;
    0 -0.0538 -0.1712 0 0.0705;
    0 0 0 1 0;
    0 0.0485 0 -0.8556 -1.013;
    0 -0.2909 0 1.0532 -0.6859];
B =[0 0 0;
    -0.12 1 0;
    0 0 0;
    4.419 0 -1.665;
    1.575 0 -0.0732];
C = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0];
D = zeros(3,3);
G = ss(A,B,C,D);

%matrix dimension
[n,n] = size(A);
[n,m] = size(B);
[p,n] = size(C);

% %weighting matrices
% %Generalised Plant_Design 1
% Ap=A;
% Bw=2*[B zeros(n,p+(n-m))];
% Bu=B;
% Bp=[Bw Bu];
% Cz=2*[C;zeros(m+(n-p),n)];
% Cv=-C;
% Cp=[Cz;Cv];
% Dzw=zeros(n+m,n+p);
% Dzu=[zeros(n,m);eye(m)];
% Dvw=[zeros(p,n) eye(p)];
% Dvu=zeros(p,m);
% Dp=[Dzw Dzu;Dvw Dvu];
% Gp1=ss(Ap,Bp,Cp,Dp);

%Generalised Plant_Design 2
Ap = A;

Bw = 50*[B zeros(n,p+(n-m))];
Bu = B;
Bp = [Bw Bu];

Cz = 50*[C;zeros(m+(n-p),n)];
Cv = -C;
Cp = [Cz;Cv];

Dzw = zeros(n+m,p+n);
Dzu = [zeros(n,m);eye(m)];
Dvw = [zeros(p,n) eye(p)];
Dvu = zeros(p,m);
Dp = [Dzw Dzu;Dvw Dvu];

Gp2 = ss(Ap,Bp,Cp,Dp);
 
%LQG controller
nmeas = 3; %no of measured otputs
ncont = 3; %no of ctrlr inputs
[K,CL] = h2syn(Gp2,nmeas,ncont);

%plots
S = inv(eye(3)+G*K); % Sensitivity
T = G*K*S; %Complementary Sensitivity
figure
sigma(T,'r',S,'b--')
grid
legend('T','S')
figure
sigma(CL)
grid