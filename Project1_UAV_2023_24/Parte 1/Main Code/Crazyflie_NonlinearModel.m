clc
clear all


%UAV´s Project

%Following the North-East-Down (NED) inertial frame

%Earth parameters
zI=[0; 0; 1];
g_earth=9.82;%m/s^2
ro_earth=1.217;%<kg/m^3
g=[0;0;g_earth]; %Earth gravity vector

%Crazyflie sides dimensions
x_dimension=0.035;%m
y_dimension=0.03;%m
z_dimension=0.019;%m

%Craziflie parameters
m=0.032;%kg massa (without payload 15g)(25g original)
W=m*g;%N
fg=W;%gravity force
Cd=1;%equal to the rectangular shape as represented in the project statement
Area=diag([y_dimension*z_dimension, x_dimension*z_dimension, y_dimension*x_dimension]);%m^2
Drag_Body=-Cd*0.5*Area*ro_earth*3;%3 is the assumed velocity that we believe it can take in the magority of times
%fa=Drag_Body*v;




%Moment of inertia matrix
J=[1.395*10^-5 0 0
    0 1.436*10^-5 0
    0 0 2.173*10^-5];%kg/m^2

%Positions fo the rotors
pos1=[3.25; -3.25; 1.45]*10^-2;
pos2=[-3.25; -3.25; 1.45]*10^-2;
pos3=[-3.25; 3.25; 1.45]*10^-2;
pos4=[3.25; 3.25; 1.45]*10^-2;%m


%Time definition
Dt=0.01; %s %time interval between each moment/iteration
t = 0:Dt:20; %time vector from 0 to 50 seconds with an interval of Dt
N = length( t ); %total number of iterations/instants or time vector length

%Thrust definition
Thrust=(m+0.015)*g_earth/4;%N %Max Thrust assumed
%Thrust=m*g_earth/4;
T=Thrust*4;

Fp1=[0;0;1]*Thrust*(t>=0);
Fp2=[0;0;1]*Thrust*(t>=0);
Fp3=[0;0;1]*Thrust*(t>=0);
Fp4=[0;0;1]*Thrust*(t>=0);
fp=Fp1+Fp2+Fp3+Fp4;


%moment vector
CQ=7.9379*10^-12;%N.m/RPM^2
CT=3.1582*10^-10;%N/RPM^2
cte=CQ/CT;
np=moment_vector_func([Thrust;Thrust;Thrust;Thrust], Fp1,Fp2,Fp3,Fp4,pos1,pos2,pos3,pos4,cte);

%Propulsion for for each rotor
u=[T;np(1);np(2);np(3)]*(t>=0);%We are not using this format for the nonlinear, we are only changing the values of the thrust of each rotor
%u=[2;2;2;2]*(t>=4); %Changes the input or each rotor thrust by 2N on the 4 seconds




%initial states
x(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % In reality the drone iself should estimate its initial state or at least position (by communicating with some reference station (it could be a controller I think)) when it starts
%Notes: the inicial velocity in the z frame cant be absolute zero becouse
%of the limitations of the pogram. Versor=v/modolo de v.



for i=1:N



  p(:,i) = x(1:3,i);
  lbd(:,i) = x(4:6,i);
  v(:,i) = x(7:9,i);
  omg(:,i) = x(10:12,i);

  R = Euler2R(lbd(:,i));
  %mod_v=(v(1,i)^2+v(2,i)^2+v(3,i)^2)^0.5;
  %versor_v=v(:,i)/mod_v;
  %fa=-0.5*ro_earth*mod_v^2*Cd*versor_v;%Air drag vector %This function should go inside the for loop since it changes with the velocity
  fa=Drag_Body*v(:,i);

  %Cinematic equation
  p_dot = R*v(:,i);
  lbd_dot = Euler2Q(lbd(:,i))*omg(:,i);
  %Dynamic equation
  v_dot = -skew(omg(:,i))*v(:,i)- g_earth*R'*zI+(1/m)*(fa+fp(:,i));
  om_dot = -inv(J)*skew(omg(:,i))*J*omg(:,i) + inv(J)*np(:,i);

  fxu=[p_dot; lbd_dot; v_dot; om_dot];
  x(:,i+1) = x(:,i) + Dt*fxu;
  y(:,i) = x(:,i);


end



figure(10);
plot(t,y(1:3,:));
grid on;
xlabel('t[s]');
ylabel('p[m]');
legend('p_x','p_y','p_z');

figure(20);
plot(t,y(4:6,:));
grid on;
xlabel('t[s]');
ylabel('lbd[rad]');
legend('fi','theta','psi');

figure(30);
plot(t,y(7:9,:));
grid on;
xlabel('t[s]');
ylabel('v[m/s]');
legend('v_x','v_y','v_z');

figure(40);
plot(t,y(10:12,:));
grid on;
xlabel('t[s]');
ylabel('w[rad/s]');
legend('w_x','w_y','w_z');

figure(50);
plot(t,u(1,:));
grid on;
xlabel('t[s]');
ylabel('T[N]');
legend('T');

figure(51);
plot(t,u(2:4,:));
grid on;
xlabel('t[s]');
ylabel('np[N.m]');
legend('np_1','np_2','np_3');



%----------------------------------------------------------------------------------------------------------
%OP1 Hover
T1_eq1=m*g_earth/4;
T2_eq1=T1_eq1;
T3_eq1=T1_eq1;
T4_eq1=T1_eq1;

Fp1_eq1=[0;0;1]*T1_eq1;
Fp2_eq1=[0;0;1]*T2_eq1;
Fp3_eq1=[0;0;1]*T3_eq1;
Fp4_eq1=[0;0;1]*T4_eq1;


T_eq1=T1_eq1+T2_eq1+T3_eq1+T4_eq1;
np_eq1=moment_vector_func([T1_eq1;T2_eq1;T3_eq1;T4_eq1], Fp1_eq1,Fp2_eq1,Fp3_eq1,Fp4_eq1,pos1,pos2,pos3,pos4,cte);
%Simulation of the linearized System

u=[T_eq1;np_eq1(1);np_eq1(2);np_eq1(3)]*(t>=0);
%u=[2;2;2;2]*(t>=4); %Changes the input or each rotor thrust by 2N on the 4 seconds

%initial states
x(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % In reality the drone iself should estimate its initial state or at least position (by communicating with some reference station (it could be a controller I think)) when it starts

%Definition of the position and porpulsion of equilibrium
xeq=[x(1:6,1); 0; 0; 0; 0; 0; 0]*(t>=0);
ueq=[T_eq1;np_eq1(1);np_eq1(2);np_eq1(3)]*(t>=0);

[A1, B1, C1, D1]=SS_matrices(xeq(1,1),xeq(2,1),xeq(3,1),xeq(4,1),xeq(5,1),xeq(6,1),xeq(7,1),xeq(8,1),xeq(9,1),xeq(10,1),xeq(11,1),xeq(12,1),g_earth, m, J(1,1), J(2,2), J(3,3), Drag_Body(1,1),Drag_Body(2,2),Drag_Body(3,3))


for i=1:N



  fxu = A1*(x(:,i)-xeq(:,i)) + B1*(u(:,i)-ueq(:,1));
  x(:,i+1) = x(:,i) + Dt*fxu;
  y(:,i) = x(:,i);

end


figure(60);
plot(t,y(1:3,:));
grid on;
xlabel('t[s]');
ylabel('p[m]');
legend('p_x','p_y','p_z');

figure(70);
plot(t,y(4:6,:));
grid on;
xlabel('t[s]');
ylabel('lbd[rad]');
legend('fi','theta','psi');

figure(80);
plot(t,y(7:9,:));
grid on;
xlabel('t[s]');
ylabel('v[m/s]');
legend('v_x','v_y','v_z');

figure(90);
plot(t,y(10:12,:));
grid on;
xlabel('t[s]');
ylabel('w[rad/s]');
legend('w_x','w_y','w_z');

figure(100);
plot(t,u(1,:));
grid on;
xlabel('t[s]');
ylabel('T[N]');
legend('T');

figure(101);
plot(t,u(2:4,:));
grid on;
xlabel('t[s]');
ylabel('np[N.m]');
legend('np_1','np_2','np_3');



%controlability, observability
if (12 - rank(ctrb(A1,B1))) > 0, disp('System is uncontrolable'); else disp('System is controlable'); end
if (12 - rank(obsv(A1,C1))) > 0, disp('System is unobservable'); else disp('System is observable'); end

%Stability
Ajj=sym(A1);
[VV,JJ] = jordan(A1);
cond = JJ == VV\A1*VV;
isAlways(cond);
if eig(A1)>0 & eig(JJ)~=1 , disp('System is unstable'); elseif eig(A1)<=0,  disp('System is marginally stable');elseif eig(A1)<0,  disp('System is exponentially stable'); end


%-----------------------------------------------------------------------------------------------------
%OP2 Horizontal flight

%initial states
x(1:3,1) = [0;0;0]; % In reality the drone iself should estimate its initial state or at least position (by communicating with some reference station (it could be a controller I think)) when it starts
x(4:6,1) = [pi/360;pi/360;0];
x(7:9,1) = [2.8827; -2.4704;0.0467]*2;
x(10:12,1) = [0;0;0];

%equilibrium inputs
T1_eq2=0.34358428178307645428157268436488/4;%Definition of the thrust
T2_eq2=T1_eq2;
T3_eq2=T1_eq2;
T4_eq2=T1_eq2;

%----------
v_eq_inercial=[2.8827672712878218444836116856714; -2.4706880189719336114995718566559;0]
p_eq=v_eq_inercial*(t>=0);
lbd_eq=[pi/360;pi/360;0]*(t>=0);
%v_eq=[2.8827; -2.4704;0.0467]*(t>=0);
v_eq=Euler2R([pi/360;pi/360;0])*v_eq_inercial*(t>=0);
om_eq=[0;0;0]*(t>=0);

%-------
%Simulation of the linearized System
Fp1_eq2=[0;0;1]*T1_eq2;
Fp2_eq2=[0;0;1]*T2_eq2;
Fp3_eq2=[0;0;1]*T3_eq2;
Fp4_eq2=[0;0;1]*T4_eq2;


T_eq2=T1_eq1+T2_eq1+T3_eq1+T4_eq1;
np_eq2=moment_vector_func([T1_eq2;T2_eq2;T3_eq2;T4_eq2], Fp1_eq2,Fp2_eq2,Fp3_eq2,Fp4_eq2,pos1,pos2,pos3,pos4,cte);
%Simulation of the linearized System

u=[T_eq2;np_eq2(1);np_eq2(2);np_eq2(3)]*(t>=0);
%u=[2;2;2;2]*(t>=4); %Changes the input or each rotor thrust by 2N on the 4 seconds


%Definition of the position and porpulsion of equilibrium
xeq=[p_eq;lbd_eq;v_eq;om_eq];
ueq=[T_eq2;np_eq2(1);np_eq2(2);np_eq2(3)]*(t>=0);


[A2, B2, C2, D2]=SS_matrices(xeq(1,1),xeq(2,1),xeq(3,1),xeq(4,1),xeq(5,1),xeq(6,1),xeq(7,1),xeq(8,1),xeq(9,1),xeq(10,1),xeq(11,1),xeq(12,1),g_earth, m, J(1,1), J(2,2), J(3,3), Drag_Body(1,1),Drag_Body(2,2),Drag_Body(3,3))


for i=1:N



  fxu = A2*(x(:,i)-xeq(:,i)) + B2*(u(:,i)-ueq(:,i));
  x(:,i+1) = x(:,i) + Dt*fxu;
  y(:,i) = x(:,i);

end


figure(110);
plot(t,y(1:3,:));
grid on;
xlabel('t[s]');
ylabel('p[m]');
legend('p_x','p_y','p_z');

figure(120);
plot(t,y(4:6,:));
grid on;
xlabel('t[s]');
ylabel('lbd[rad]');
legend('fi','theta','psi');

figure(130);
plot(t,y(7:9,:));
grid on;
xlabel('t[s]');
ylabel('v[m/s]');
legend('v_x','v_y','v_z');

figure(140);
plot(t,y(10:12,:));
grid on;
xlabel('t[s]');
ylabel('w[rad/s]');
legend('w_x','w_y','w_z');

figure(150);
plot(t,u(1,:));
grid on;
xlabel('t[s]');
ylabel('T[N]');
legend('T');

figure(151);
plot(t,u(2:4,:));
grid on;
xlabel('t[s]');
ylabel('np[N.m]');
legend('np_1','np_2','np_3');

%controlability, observability
if (12 - rank(ctrb(A2,B2))) > 0, disp('System is uncontrolable'); else disp('System is controlable'); end
if (12 - rank(obsv(A2,C2))) > 0, disp('System is unobservable'); else disp('System is observable'); end

%Stability
Ajj=sym(A2);
[VV,JJ] = jordan(A2);
cond = JJ == VV\A2*VV;
isAlways(cond);
if eig(A2)>0 & eig(JJ)~=1 , disp('System is unstable'); elseif eig(A2)<=0,  disp('System is marginally stable');elseif eig(A2)<0,  disp('System is exponentially stable'); end


%Transfer functions and Statespace models
sysHov = ss(A1,B1,C1,D1); 
Gl_Hov=tf(sysHov);  
sysHori = ss(A2,B2,C2,D2); 
Gl_Hori=tf(sysHori);

GGl_Hov_p_110=[Gl_Hov(4,2), Gl_Hov(2,2)/Gl_Hov(4,2);
              Gl_Hov(5,3), Gl_Hov(1,3)/Gl_Hov(5,3);
              Gl_Hov(6,4), Gl_Hov(3,1)]
      
Gl_Hori_p_110=[Gl_Hori(4,2), Gl_Hori(2,2)/Gl_Hori(4,2);
              Gl_Hori(5,3), Gl_Hori(1,3)/Gl_Hori(5,3);
              Gl_Hori(6,4), Gl_Hori(3,1)]
           
%Root-locus
figure(160);
rlocus(GGl_Hov_p_110(3,2));

figure(170);
rlocus(-GGl_Hov_p_110(1,2));

figure(180);
rlocus(Gl_Hori_p_110(3,2));

figure(190);
rlocus(-Gl_Hori_p_110(1,2));


disp("Zeros of Hover G_T_pz")
zero(GGl_Hov_p_110(3,2))
disp("Poles of Hover G_T_pz")
pole(GGl_Hov_p_110(3,2))

disp("Zeros of Hover G_fi_py")
zero(GGl_Hov_p_110(1,2))
disp("Poles of Hover G_fi_py")
pole(GGl_Hov_p_110(1,2))

disp("Zeros of Horizontal G_T_pz")
zero(Gl_Hori_p_110(3,2))
disp("Poles of Horizontal G_T_pz")
pole(Gl_Hori_p_110(3,2))

disp("Zeros of Horizontal G_fi_py")
zero(Gl_Hori_p_110(1,2))
disp("Poles of Horizontal G_fi_py")
pole(Gl_Hori_p_110(1,2))



