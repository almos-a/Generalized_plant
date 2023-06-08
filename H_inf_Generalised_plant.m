% Lecture 17 example
clc
clear
close all
% state space model
n = 200;
d = conv([10 1],conv([0.05 1],[0.05 1]));
G = tf(n,d);
[Ag,Bg,Cg,Dg] = tf2ss(n,d);

%design of filters
M = 1.5; w0 = 10; A = 1e-4;
nws = [1/M w0];
dws = [1 w0*A]; %den check for A only
Ws = tf(nws,dws);
[Aws,Bws,Cws,Dws] = tf2ss(nws,dws);

Wks = 1;

%% Alt 1
systemnames = 'G Ws Wks';
inputvar = '[r(1);u(1)]'; % all i/p are scalar, r(2) wd be 2dim sig)
outputvar = '[Ws;Wks;r-G]';
input_to_G = '[u]';
input_to_Ws = '[r-G]';
input_to_Wks = '[u]';
sysoutname = 'P1';
cleanupsysic = 'yes';
sysic

%% Alt 2
Wt = [];
P2 = augw(G,Ws,Wks,Wt);

%% Alt 3
n1 = size(Ag,1);
n2 = size(Aws,1);
Ap3 = [Ag zeros(n1,n2);-Bws*Cg Aws];
Bw = [zeros(n1,1);Bws];
Bu = [Bg;zeros(n2,1)];
Bp3 = [Bw Bu];
Cz = [-Dws*Cg Cws;zeros(1,n1+n2)];
Cy = [-Cg zeros(1,n2)];
Cp3 = [Cz;Cy];
Dzw = [Dws;0];
Dzu = [0;1];
Dyw = [1];
Dyu = 0;
Dp3 = [Dzw Dzu;Dyw Dyu];
P3 = ss(Ap3,Bp3,Cp3,Dp3);

%% synthesizing the controller
nmeas = 1;
nu = 1;
gmn = 0.5;
gmx = 20;
tol = 0.001;
opt = hinfsynOptions('RelTol',tol);
[K,CL,gopt] = hinfsyn(P2,nmeas,nu,[gmn,gmx],opt); % either P1, P2 or P3 for generalised plant
% [K,CL,gopt]=hinfsyn(P2,nmeas,nu,gmn,gmx,tol);

%sensitivity
S = 1/(1+G*K);
%complementary sensitivity
T = G*K*S;
%input to G
KS = K*S;
%loop transfer function
GK = G*K;

figure
step(T)
grid
figure
sigma(S,1/Ws)
grid
figure
sigma(KS)
grid