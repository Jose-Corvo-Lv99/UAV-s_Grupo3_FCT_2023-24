clc
clear all

%NonLinear Control
%2.1
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


%Lyapunov V(x)=1/2*(x_1)^2+1/2*(x_2)^2
%V_dot(x)=x_1*x_dot_1+x_2*x_dot_2<=0, x!=0, Estavel
%V_dot(x)=x_1*x_dot_1+x_2*x_dot_2<0, x!=0, Assintoticament Estavel


% zB = [theta;-phi;1];
% p_dot = v;
% v_dot = -g*zI-drag*v+T/m*zB;
% vd = [0;0;2];
% 
% for i=1:Nsim
%     f = T*zB;
%     vd_dot = f/m;
%     ev = v - vd;
% 
%     ep_dot = ev;
%     ev_dot = -g*zI - drag*v + 1/m*f - vd_dot;
% end

% initialize variables for all drones:
Tend = 70;
dTo = 0.1;
dTi = 0.05;
Nsim = round(Tend/dTi)+1;
p_ref_static = [0.5;0.5;1];

t = 0:dTi:Tend;
nt = length(t);
nx = 6;
nu = 4;

p0 = [0;0;0];
v0 = [0;0;0];
x = zeros(nx,Nsim);
T = zeros(1,Nsim);
x(:,1) = [p0;v0];

% Gains for initernal controller
kp=0.2; kv=0.23;

model=2;
if model==1
 %step reference
 %p_ref = [zeros(3,20/dTi+1),p_ref_static*ones(1,(Tend-20)/dTi)];
 p_ref =[zeros(3,4),p_ref_static*ones(1,nt-4)];
 v_ref = zeros(3,nt);
 a_ref = zeros(3,nt);
elseif model==2
% circle reference
Rad = 0.5;      % radius of circle
omn = 2*pi/20;  % rotation frequency

p_ref = [Rad*cos(omn*t);Rad*sin(omn*t);ones(size(t))];
v_ref = [-Rad*omn*sin(omn*t);Rad*omn*cos(omn*t);0*ones(size(t))];
a_ref = [-Rad*omn^2*cos(omn*t);-Rad*omn^2*sin(omn*t);0*ones(size(t))];
end

% main time loop for simulation
for k = 1:Nsim

    % get state vector and plot it
    p = x(1:3,k);
    v = x(4:6,k);
    p_d = p_ref(:,k);
    v_d = v_ref(:,k);
    a_d = a_ref(:,k);

    % outer-loop controller
    e_p = p - p_d;
    e_v = v - v_d;

    % Mellinger Controller (up to attitude commands)
    f_des = -kp*e_p - kv*e_v + m*g_earth*zI + m*a_d;

    % compute desired rotation matrix
    zB_des(:,k) = f_des/norm(f_des);

    % compute thrust
    %T(:,k) = f_des'*zB_des(:,k);
    T(:,k) = max(min(0.35,f_des'*zB_des(:,k)),0);
    
    
    % nonlinear drone model
    dot_p = v;
    dot_v = -g_earth*zI - Drag_Body*v + T(:,k)/m*zB_des(:,k);

    % discretization 
    pp = p + dTi*dot_p;
    vp = v + dTi*dot_v;
    if k~=Nsim
        x(:,k+1) = [pp;vp];
    end
end

p = x(1:3,:);

% drone_show_data;
% show results plot
set(0,'defaultTextInterpreter','latex');
set(0,'defaultLegendInterpreter','latex');
sstblue         = [0,128,255]/255;
sstlightblue    = [48,208,216]/255;
sstlighterblue  = [50,220,240]/255;
sstlightestblue = [60,230,255]/255;
sstgreen        = [43,191,92]/255;
sstlightgreen   = [140,255,200]/255;
sstlightergreen = [160,255,225]/255;
sstgray         = [70,70,70]/255;
sstlightgray    = [200,200,200]/255;

dcolors = { sstgreen, sstblue, sstlightblue, sstlighterblue, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };

%angle plots
figure(101);

subplot(311);
plot(t,T,'Color',dcolors{1});
hold on;
grid on;
ylabel('$$T(t)$$ [N]');
title('Control variables');

subplot(312);
plot(t,-zB_des(2,:),'Color',dcolors{1}); %plots phi
hold on;
grid on;
ylabel('$$\phi(t)$$ [rad]');

subplot(313);
plot(t,zB_des(1,:),'Color',dcolors{1}); %plots theta
hold on;
grid on;
ylabel('$$\theta(t)$$ [rad]');
xlabel('$$t$$ [s]');

%position plots
figure(102);

subplot(311);
plot(t,p_ref(1,:),'Color',sstgray);
hold on;
plot(t,x(1,:),'Color',dcolors{1});
hold off;
grid on;
ylabel('$$x(t)$$ [m]');
title('Drone position and reference');

subplot(312);
plot(t,p_ref(2,:),'Color',sstgray);
hold on;
plot(t,x(2,:),'Color',dcolors{1});
hold off;
grid on;
ylabel('$$y(t)$$ [m]');

subplot(313);
plot(t,p_ref(3,:),'Color',sstgray);
hold on;
plot(t,x(3,:),'Color',dcolors{1});
hold off;
grid on;
xlabel('$$t$$ [s]');
ylabel('$$z(t)$$ [m]');

%drone_animate(p,p_ref,[-zB_des(2,:);zB_des(1,:);zeros(1,length(zB_des))],t,dcolors);
lbd = [-zB_des(2,:);zB_des(1,:);zeros(1,length(zB_des))];

sstgray = [70,70,70]/255;
nt = length(t);

figure(104);

hini = plot3(p(1,1),p(2,1),p(3,1),'o','Color',dcolors{1},'MarkerSize',2);
hold on;
href = plot3(p_ref(1,:),p_ref(2,:),p_ref(3,:),'--','Color',sstgray);
hp = plot3(p(1,1:2),p(2,1:2),p(3,1:2),'-','Color',dcolors{1});
hd = drone_plot(p(1:3,1),lbd(:,1),[],dcolors{1});

hold off;
grid on;
axis equal;
axis([-1.2 1.2 -1.2 1.2 0 3]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('start','end','trajectory');
for k = 2:2:nt
    set(hp,'XData',p(1,1:k),'YData',p(2,1:k),'ZData',p(3,1:k));
    drone_plot(p(1:3,k),lbd(:,k),hd);

    axis equal;
    drawnow;
    %pause(dt/10);
end