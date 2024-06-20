% NOVA School of Science and Technology
% Department of Electrical and Computer Engineering
% 2022
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% inicializations

% Model and simulation parameters
Tend = 70;
dTo = 0.1;
dTi = 0.05;
Nsim = round(Tend/dTi)+1;
nD = 1; % number of drones
use_MPC = [0 0*ones(1,nD-1)];
dh = 0.05;      % safety height difference between drones
Rad = 0.5;      % radius of circle
omn = 2*pi/20;  % rotation frequency
dphase = -pi/12;% ref circle angular difference between drones

% Gains for initernal controller
kp=0.39; kv=0.29; ki = 0.0; kR = 3; kom=0.5; 

ref_mode = 1;

% Crazyflie 2.0 based on 
% Benoit Landry, "Planning and Control for Quadrotor Flight through Cluttered Environments", BSc, MIT, 2014
m = 0.031;      % mass (added board)
I = diag([2.3951e-5,2.3951e-5,3.2347e-5]);  % inertia tensor
D = 0.001;      % frame drag coeficient
g = 9.8;

% initial conditions
p_ref_static = [0.5;0.5;1];
psi_ref_static = pi/3;

% put parameters into structure
Param.dTo = dTo;% outer-loop sampling period
Param.dTi = dTi;% inner-loop and simulation sampling period
Param.g = g;    % earth gravity
Param.m = m;    % drone mass
Param.I = I;    % drone inertia tensor
Param.D  = D;   % drone fuselage drag force coeficient
Param.kp = Param.m/Param.dTi*diag([kp/2,kp/2,kp]);
Param.kv = Param.m/Param.dTi*diag([kv/2,kv/2,kv]);
Param.ki = Param.m/Param.dTi*diag([ki,ki,ki/2]);
Param.kR = Param.I/Param.dTi*diag([kR,kR,kR*4]);
Param.kom= Param.I/Param.dTi*diag([kom,kom,kom*4]);
Param.p_ref_static = p_ref_static;

%%%%%%%%%% compute MPC parameters
MPC.Pi = 4; %notice P has dimensions compatible with the output
MPC.Qi = 1; %notice Q has dimensions compatible with the output
MPC.Ri = 10;
MPC.alphai = 1;
MPC.N = 30;
MPC.nD = nD;
% distributed steps parameters
MPC.wi = 1/nD;
MPC.np = 2;

%etc...