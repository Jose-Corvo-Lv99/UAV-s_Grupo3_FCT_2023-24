clear all
%Linear Control


%1.1. 


%Earth parameters
zI=[0; 0; 1];
g_earth=9.82;%m/s^2
ro_earth=1.217;%<kg/m^3



%Crazyflie sides dimensions
x_dimension=0.035;%m
y_dimension=0.03;%m
z_dimension=0.019;%m

%Craziflie parameters
m=0.032;%kg massa (without payload 15g)(25g original)

%Moment of inertia matrix
J=[1.395*10^-5 0 0
    0 1.436*10^-5 0
    0 0 2.173*10^-5];%kg/m^2

g=[0;0;g_earth]; %Earth gravity vector
Cd=1;%equal to the rectangular shape as represented in the project statement
Area=diag([y_dimension*z_dimension, x_dimension*z_dimension, y_dimension*x_dimension]);%m^2

Drag_Body=-Cd*0.5*Area*ro_earth*3;%3 is the assumed velocity that we believe it can take in the magority of times


%Time definition
Dt=0.01; %s %time interval between each moment/iteration
t = 0:Dt:5; %time vector from 0 to 50 seconds with an interval of Dt
N = length( t ); %total number of iterations/instants or time vector length

T = ones(1,N);

roll=zeros(1,N);
pitch =zeros(1,N);
yaw=zeros(1,N); %yaw=0 vai ser 0 segundo 

u_lbd=[T; roll; pitch; yaw]; 

p=zeros(3,N);
v_ine=zeros(3,N);
p(:,1)=[0; 0; 0];
v_ine(:,1)=[0; 0; 0];
x(:,1)=[p(:,1); v_ine(:,1)];

%1.2.
ua(:,1) = [0; 0; 0];

%1.3.
xLin(:,1)=x(:,1);

A = [0 0 0 1 0 0
     0 0 0 0 1 0
     0 0 0 0 0 1
     0 0 0 Drag_Body(1,1) 0 0
     0 0 0 0 Drag_Body(2,2) 0
     0 0 0 0 0 Drag_Body(3,3)];


B = [0 0 0 
     0 0 0
     0 0 0
     1 0 0
     0 1 0
     0 0 1];



for i = 1:N
    
    fa_ine=Drag_Body*x(4:6,i);   
    R = Euler2R(u_lbd(2:4,i));
    fp=u_lbd(1,i)*[0; 0; 1];
    
    %Cinematic equation
    
    p_dot = x(4:6,i);

    
    %Dynamic equation
    v_ine_dot = - g_earth*zI+(1/m)*(fa_ine+R*fp);
  
    
    

    fxu=[p_dot; v_ine_dot];
    x(:,i+1) = x(:,i) + Dt*fxu;
    %y(1:3,i) = x(1:3,i);
    %y(4:6,i) = R'*x(4:6,i);
    y(:,i) = x(:,i);

    
    
    %%1.2.
    ua(:,i) = v_ine_dot-(- g_earth*zI+(1/m)*fa_ine);
   
    %1.3.
    fxuLin = A*xLin(:,i) + B*ua(:,i); 
    xLin(:,i+1) = xLin(:,i) + Dt*fxuLin;
    yLin(:,i) = xLin(:,i);
   

end

figure(80);
plot(t,y(1:3,:));
hold on
plot(t,yLin(1:3,:))

hold off
grid on;
xlabel('t[s]');
ylabel('p[m]');
legend('p_x1','p_y1','p_z1','p_x2','p_y2','p_z2');



figure(90);
plot(t,y(4:6,:));
hold on
plot(t,yLin(4:6,:))

hold off
grid on;
xlabel('t[s]');
ylabel('v[m/s]');
legend('v_x1','v_y1','v_z1', 'v_x2','v_y2','v_z2');



figure(100);
plot(t,u_lbd(1,:));
grid on;
xlabel('t[s]');
ylabel('T[N]');
legend('T');

figure(110);
plot(t,u_lbd(2:4,:));
grid on;
xlabel('t[s]');
ylabel('º[deg]');
legend('roll','pitch','yaw');

%%1.2.
figure(120);
plot(t,ua(1:3,:));
grid on;
xlabel('t[s]');
ylabel('a[m/s^2]');
legend('ua_x','ua_y','ua_z');
  









%1.3.
% v_ine_dot = - g_earth*zI+(1/m)*fa_ine + ua_ine = aceleraçoes inerciais 
% p_dot = x(4:6,i);
%[A,B,Q,R] = state_space_matrix_func (0, 0, 0);


%1.4
%Q = eye(6);
Q = diag([ 10; 10; 10; 0.0001; 0.0001; 0.00000001]);
%R = 0.1 * eye(3);
R = diag([ 1; 1; 0.01]);
[K,S,P]=lqr(A,B,Q,R);





ref(1:3,:) = ones(3, length(t))*0;
ref(4:6,:) = ones(3, length(t))*0.000001;


p=zeros(3,N);
v_ine=zeros(3,N);
p(:,1)=[0; 0; 10];
v_ine(:,1)=[0; 0; 10];
x(:,1)=[p(:,1); v_ine(:,1)];

xLin(:,1)=x(:,1);


y(:,1)=[0; 0; 10; 0; 0; 0];
yLin(:,1)=[0; 0; 10; 0; 0; 0];

for i = 1:N
    
    fa_ine=Drag_Body*x(4:6,i); 
    
    e = ref(:, i)-y(:,i);
    eLin = ref(:, i)-yLin(:,i);

    u = K*e;
    uLin =K*eLin;

    %fa_ine=Drag_Body*x(4:6,i);   
    %R = Euler2R(u_lbd(2:4,i));
    %fp=u()*[0; 0; 1];
    
    v_ine_dot =  - g_earth*zI+(1/m)*fa_ine + u ;
    p_dot = x(4:6,i);
    
    
    
    fxu=[p_dot; v_ine_dot];
    x(:,i+1) = x(:,i) + Dt*fxu;
    y(:,i+1) = x(:,i);

    
    
    %1.3.
    fxuLin = (A-B*K)*xLin(:,i); %+ B*uLin; 
    xLin(:,i+1) = xLin(:,i) + Dt*fxuLin;
    yLin(:,i+1) = xLin(:,i);


end

figure(130);
plot(t,y(1:3,1:length(t)));
hold on
plot(t, ref(1:3,:))
hold off
grid on;
xlabel('t[s]');
ylabel('p[m]');
legend('p_x1','p_y1','p_z1', 'ref1', 'ref2', 'ref3');

figure(131);
plot(t,yLin(1:3,1:length(t)))
hold on
plot(t, ref(1:3,:))
hold off
grid on;
xlabel('t[s]');
ylabel('p[m]');
legend('p_x2','p_y2','p_z2', 'ref1', 'ref2', 'ref3');


figure(140);
plot(t,y(4:6,1:length(t)));
hold on
plot(t, ref(4:6,:))
hold off
grid on;
xlabel('t[s]');
ylabel('v[m/s]');
legend('v_x1','v_y1','v_z1','ref1', 'ref2', 'ref3');


figure(141);
plot(t,yLin(4:6,1:length(t)))
hold on
plot(t, ref(4:6,:))
hold off
grid on;
xlabel('t[s]');
ylabel('v[m/s]');
legend('v_x2','v_y2','v_z2','ref1', 'ref2', 'ref3');




%sys = ss(A,B,eye(6), zeros(6,3));
%sys = ss(A-B*K,B,eye(6), zeros(6,3))
%G = tf(sys)

%H = G*K*(ones(6)+G*K)^-1

%sys1 = ss(A-B*K,B,eye(6), zeros(6,3));
%step(sys1)
 


Ae = [0 0 0 1 0 0
      0 0 0 0 1 0
      0 0 0 0 0 1
      0 0 0 0 0 0
      0 0 0 0 0 0
      0 0 0 0 0 0];

Be = [0 0 0
      0 0 0 
      0 0 0 
      1 0 0
      0 1 0
      0 0 1];



Q = diag([ 10; 10; 10; 0.0001; 0.0001; 0.00000001]);
%R = 0.1 * eye(3);
R = diag([ 1; 1; 0.01]);
[K,S,P]=lqr(Ae,Be,Q,R);





ref(1:3,:) = ones(3, length(t))*0;
ref(4:6,:) = ones(3, length(t))*0;


p=zeros(3,N);
v_ine=zeros(3,N);
ep(:,1)=[10; 10; 10];
ev_ine(:,1)=[0; 0; 0];
x(:,1)=[ep(:,1); ev_ine(:,1)];

xLin(:,1)=x(:,1);


y(:,1)=[10; 10; 10; 0; 0; 0];
yLin(:,1)=[10; 10; 10; 0; 0; 0];

for i = 1:N
    
    
   
    
    
    e = ref(:, i)-y(:,i);
    eLin = ref(:, i)-yLin(:,i);

    u = K*e;
    uLin = K*eLin;
    
%     u = u-g_earth;
%     uLin = uLin+g_earth;

    %fa_ine=Drag_Body*x(4:6,i);   
    %R = Euler2R(u_lbd(2:4,i));
    %fp=u()*[0; 0; 1];
    
    ev_ine_dot = u;
    ep_dot = x(4:6,i);
    
    
    
    fxu=[ep_dot; ev_ine_dot];
    x(:,i+1) = x(:,i) + Dt*fxu;
    y(:,i+1) = x(:,i);

    
    
    %1.3.
    fxuLin = Ae*xLin(:,i) + Be*uLin; 
    xLin(:,i+1) = xLin(:,i) + Dt*fxuLin;
    yLin(:,i+1) = xLin(:,i);


end

figure(150);
plot(t,y(1:3,1:length(t)));
hold on
plot(t, ref(1:3,:))
hold off
grid on;
xlabel('t[s]');
ylabel('ep[m]');
legend('ep_x1','ep_y1','ep_z1', 'ref1', 'ref2', 'ref3');

figure(151);
plot(t,yLin(1:3,1:length(t)))
hold on
plot(t, ref(1:3,:))
hold off
grid on;
xlabel('t[s]');
ylabel('ep[m]');
legend('ep_x2','ep_y2','ep_z2', 'ref1', 'ref2', 'ref3');


figure(160);
plot(t,y(4:6,1:length(t)));
hold on
plot(t, ref(4:6,:))
hold off
grid on;
xlabel('t[s]');
ylabel('ev[m/s]');
legend('ev_x1','ev_y1','ev_z1','ref1', 'ref2', 'ref3');


figure(161);
plot(t,yLin(4:6,1:length(t)))
hold on
plot(t, ref(4:6,:))
hold off
grid on;
xlabel('t[s]');
ylabel('ev[m/s]');
legend('ev_x2','ev_y2','ev_z2','ref1', 'ref2', 'ref3');




ref(1:3,:) = ones(3, length(t))*10;
ref(4:6,:) = ones(3, length(t))*0;


p=zeros(3,N);
v_ine=zeros(3,N);
ep(:,1)=[0; 0; 0];
ev_ine(:,1)=[0; 0; 0];
x(:,1)=[ep(:,1); ev_ine(:,1)];

xLin(:,1)=x(:,1);


y(:,1)=[0; 0; 0; 0; 0; 0];
yLin(:,1)=[0; 0; 0; 0; 0; 0];

for i = 1:N
    
    
   
    
    
    e = ref(:, i)-y(:,i);
    eLin = ref(:, i)-yLin(:,i);

    u = K*e;
    uLin = K*eLin;
    
%     u = u-g_earth;
%     uLin = uLin+g_earth;

    %fa_ine=Drag_Body*x(4:6,i);   
    %R = Euler2R(u_lbd(2:4,i));
    %fp=u()*[0; 0; 1];
    
    ev_ine_dot = - g_earth*zI+(1/m)*fa_ine + u+g_earth*zI;
    ep_dot = x(4:6,i);
    
    
    
    fxu=[ep_dot; ev_ine_dot];
    x(:,i+1) = x(:,i) + Dt*fxu;
    y(:,i+1) = x(:,i);

    
    
    %1.3.
    fxuLin = A*xLin(:,i) + B*uLin; 
    xLin(:,i+1) = xLin(:,i) + Dt*fxuLin;
    yLin(:,i+1) = xLin(:,i);


end

figure(170);
plot(t,y(1:3,1:length(t)));
hold on
plot(t, ref(1:3,:))
hold off
grid on;
xlabel('t[s]');
ylabel('p[m]');
legend('p_x1','p_y1','p_z1', 'ref1', 'ref2', 'ref3');

figure(171);
plot(t,yLin(1:3,1:length(t)))
hold on
plot(t, ref(1:3,:))
hold off
grid on;
xlabel('t[s]');
ylabel('p[m]');
legend('p_x2','p_y2','p_z2', 'ref1', 'ref2', 'ref3');


figure(180);
plot(t,y(4:6,1:length(t)));
hold on
plot(t, ref(4:6,:))
hold off
grid on;
xlabel('t[s]');
ylabel('v[m/s]');
legend('v_x1','v_y1','v_z1','ref1', 'ref2', 'ref3');


figure(181);
plot(t,yLin(4:6,1:length(t)))
hold on
plot(t, ref(4:6,:))
hold off
grid on;
xlabel('t[s]');
ylabel('v[m/s]');
legend('v_x2','v_y2','v_z2','ref1', 'ref2', 'ref3');

%%
