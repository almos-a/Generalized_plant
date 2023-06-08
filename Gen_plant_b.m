% Generalised plant without r
clc
clear
% Aircraft model
A = [0 0 1.132 0 -1;
    0 -0.0538 -0.1712 0 0.0705;
    0 0 0 1 0;
    0 0.0485 0 -0.8556 -1.013;
    0 -0.2909 0 1.0532 -0.6859];
B = [0 0 0;
    -0.12 1 0;
    0 0 0;
    4.419 0 -1.665;
    1.575 0 -0.0732];
C = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0];
D = zeros(3,3);
G = ss(A,B,C,D);

Q = 0.1*eye(5);
R = 0.01*eye(3);
Qe = eye(5);
Re = 0.001*eye(3);

% %matrix dimension
% [n,n] = size(A);
% [n,m] = size(B);
% [p,n] = size(C);

%Generalised Plant_State space mode
Ap = A;
Bp = [[sqrtm(Qe) zeros(5,3)] B];
Cp = [[sqrtm(Q);zeros(3,5)];-C];
Dp = [zeros(8,8) [zeros(5,3);sqrtm(R)];[zeros(3,5) -sqrtm(Re)] zeros(3,3)];
Gp = ss(Ap,Bp,Cp,Dp);

%LQG controller
nmeas = 3; %no of measured otputs
ncont = 3; %no of ctrlr inputs

[K,CL,GAM,INFO] = h2syn(Gp,nmeas,ncont)
%state space data of controller
[a,b,c,d] = ssdata(K);

%step response
% senstivity
S = inv(eye(3)+G*K);
% controller senstivity
SK = K*S;
figure
step(SK(1,1),SK(2,1),SK(3,1))
grid
ylabel('input u');xlabel('time');
title('Time response');
legend('u1','u2','u3')

% complementary sensitivity
T = G*K*S;
figure
step(T(1,1),T(2,1),T(3,1))
grid
ylabel('input u');xlabel('time');
title('Time response');