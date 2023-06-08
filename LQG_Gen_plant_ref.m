% Generalised plant with reference input
clc
clear
% model
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

%matrix dimension
[n,n] = size(A);
[n,m] = size(B);
[p,n] = size(C);

Q = eye(n);
R = 0.01*eye(m);
Qe = eye(n);
Re = 0.001*eye(p);

%Generalised Plant_State space model with r inc
Ap = A;

B1 = [zeros(n,p) sqrtm(Qe) zeros(n,p)];
B2 = B;
Bp = [B1 B2];

C1 = [sqrtm(Q);zeros(m,n)];
C2 = -C;
Cp = [C1;C2];

D11 =  zeros(n+m,p+n+p);
D12 = [zeros(n,m);sqrtm(R)];
D21 = [eye(p) zeros(p,n) -sqrtm(Re)];
D22 = zeros(p,m);
Dp = [D11 D12;D21 D22];

Gp = ss(Ap,Bp,Cp,Dp);

%LQG controller
nmeas = 3; %no of measured otputs
ncont = 3; %no of ctrlr inputs
[K,CL,info] = h2syn(Gp,nmeas,ncont);

H2 = norm(CL,2)
Hinf = norm(CL,inf)

% norm
S = inv(eye(3)+G*K); % Sensitivity
T = G*K*S; %Complementary Sensitivity

T2 = norm(T,2)
Tinf = norm(T,inf)

% H-inf
figure
sigma(T)
grid
figure
sigma(CL)
grid
% T ss
[At,Bt,Ct,Dt] = ssdata(T);
M = [At Bt*Bt';-Ct'*Ct -At'];
eig(M)
rank(ctrb(At,Bt))
rank(obsv(At,Ct))

[At,Bt,Ct,Dt] = ssdata(CL);
M = [At Bt*Bt';-Ct'*Ct -At'];
eig(M)
rank(ctrb(At,Bt))
rank(obsv(At,Ct))