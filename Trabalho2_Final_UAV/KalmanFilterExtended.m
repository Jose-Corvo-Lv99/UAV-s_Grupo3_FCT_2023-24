% crazyflie load usd card log csv file for sensor analysis
clear all

% available files:
csvfilename_motors_off = 'L2Data1.csv';
csvfilename_hover = 'L2Data2.csv';
csvfilename_motion_x = 'L2Data3.csv';
csvfilename_motion_y = 'L2Data4.csv';
csvfilename_motion_z = 'L2Data5.csv';
csvfilename_motion_inf = 'L2Data6.csv';

% read file
csvfilename = csvfilename_motion_inf ;
array = dlmread(csvfilename,',',1,0);
%T = table2array(readtable(csvfilename)); % Matlab only

% get data from table
time = array(:,1)'*1e-3;
pos = array(:,2:4)'; % [m]
vel = array(:,5:7)'; % [m/s]
lbd = array(:,8:10)'; % [deg]
gyro = array(:,11:13)'; % [deg/s]
acc = array(:,14:16)'; % [Gs]
baro_asl = array(:,17)'; % [m]
% lighthouse = array(:,18:20))'; % [m]

% convert date to print format
t = time - time(1);




% Caracterização Estatística

%Média
posX_avg = mean_function(pos(1,:))
posY_avg = mean_function(pos(2,:))
posZ_avg = mean_function(pos(3,:))

velX_avg = mean_function(vel(1,:))
velY_avg = mean_function(vel(2,:))
velZ_avg = mean_function(vel(3,:))

roll_avg = mean_function(lbd(1,:))
theta_avg = mean_function(lbd(2,:))
yaw_avg = mean_function(lbd(3,:))

omgX_avg = mean_function(gyro(1,:))
omgY_avg = mean_function(gyro(2,:))
omgZ_avg = mean_function(gyro(3,:))

accX_avg = mean_function(acc(1,:))
accY_avg = mean_function(acc(2,:))
accZ_avg = mean_function(acc(3,:))

baro_asl_avg = mean_function(baro_asl)


%Covariância
posX_cov = covariance_function(pos(1,:), pos(1,:))
posY_cov = covariance_function(pos(2,:), pos(2,:))
posZ_cov = covariance_function(pos(3,:), pos(3,:))

velX_cov = covariance_function(vel(1,:), vel(1,:))
velY_cov = covariance_function(vel(2,:), vel(2,:))
velZ_cov = covariance_function(vel(3,:), vel(3,:))

roll_cov = covariance_function(lbd(1,:), lbd(1,:))
theta_cov = covariance_function(lbd(2,:), lbd(2,:))
yaw_cov = covariance_function(lbd(3,:), lbd(3,:))

omgX_cov = covariance_function(gyro(1,:), gyro(1,:))
omgY_cov = covariance_function(gyro(2,:), gyro(2,:))
omgZ_cov = covariance_function(gyro(3,:), gyro(3,:))

accX_cov = covariance_function(acc(1,:), acc(1,:))
accY_cov = covariance_function(acc(2,:), acc(2,:))
accZ_cov = covariance_function(acc(3,:), acc(3,:))

baro_asl_cov = covariance_function(baro_asl, baro_asl)

avg_matrix=[posX_avg, velX_avg, roll_avg, omgX_avg, accX_avg;
            posY_avg, velY_avg, theta_avg, omgY_avg, accY_avg;
            posZ_avg, velZ_avg, yaw_avg, omgZ_avg, accZ_avg]
baro_asl_avg


cov_matrix=[posX_cov, velX_cov, roll_cov, omgX_cov, accX_cov;
            posY_cov, velY_cov, theta_cov, omgY_cov, accY_cov;
            posZ_cov, velZ_cov, yaw_cov, omgZ_cov, accZ_cov]
baro_asl_cov

%%%%%% Calculo dos Roll e Pitch através do Acelerómetro
g = 1;
ax = acc(1,:);
ay = acc(2,:);
az = acc(3,:);




%roll = atan2(ay, az);   %rad
%pitch = asin(ax, g);    %rad

roll = arrayfun( @(x,y) atan2(x,y), ay, az); %rad
pitch = arrayfun( @(x) asin(x), ax.*((g*ones(1, length(ax))).^-1) ); %rad

Nsim = length(t);
roll_gyros=zeros(1,Nsim);
roll_gyros(1)=roll(1)*180/pi;
pitch_gyros=zeros(1,Nsim);
pitch_gyros(1)=pitch(1)*180/pi;
yaw_gyros=zeros(1,Nsim);
yaw_gyros(1)=lbd(3,1);
lbd_gyros=zeros(3,Nsim);
lbd_gyros_Der=zeros(3,Nsim);
for k=2:Nsim
    
    roll_gyros(k)=gyro(1,k)*0.004+roll_gyros(k-1);
    pitch_gyros(k)=-gyro(2,k)*0.004+pitch_gyros(k-1);
    
%     lbd_gyros_Der(:,k)=Euler2Q([roll_gyros(k-1) pitch_gyros(k-1) yaw_gyros(k-1)])*gyro(:,k)*pi/180;
%     lbd_gyros(k)=lbd_gyros_Der(:,k)*0.004+lbd_gyros(:,k-1);
%     roll_gyros(k)=lbd_gyros(1,k);
%     pitch_gyros(k)=lbd_gyros(2,k);
%     yaw_gyros(k)=lbd_gyros(3,k);
end

figure(50);
subplot(2,1,1);
plot(t,roll_gyros, t, lbd(1,:)); %assim é como faz mais sentido com isto ao quadrado
grid on;
xlabel('t [s]');
ylabel('$$\phi$$ [rad]');
legend('sensors','given');
title('roll angle estimates')

subplot(2,1,2);
plot(t,pitch_gyros, t, lbd(2,:));
grid on;
xlabel('t [s]');
ylabel('$$\theta$$ [rad]');
legend('sensors','given');
title('pitch angle estimates')

roll_gyros=roll_gyros*pi/180;
pitch_gyros=pitch_gyros*pi/180;
mean_roll_gyros=mean_function(roll_gyros);
cov_roll_gyros=covariance_function(roll_gyros, roll_gyros)
mean_pitch_gyros=mean_function(pitch_gyros);
cov_pitch_gyros=covariance_function(pitch_gyros, pitch_gyros)


%Filtro de Kalman Extendido
%com bias

A1=[0 1 0 0 0 0;
   0 0 0 0 0 0;
   0 0 0 1 0 0;
   0 0 0 0 0 0;
   0 0 0 0 0 0;
   0 0 0 0 0 0];

B1=[0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];

C1=[1 0 0 0 1 0;
    0 0 1 0 0 1];

%D1=[0, 0, 0, 0;
%    0, 0, 0, 0];


%Sem bias

A2=[0 1 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 0 0];

B2=[0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];

C2=[1 0 0 0;
    0 0 1 0];

%D2=[0, 0, 0, 0;
%    0, 0, 0, 0];

A3=[0 1 0 0 0 0;
   0 0 0 0 0 0;
   0 0 0 1 0 0;
   0 0 0 0 0 0;
   0 0 0 0 0 0;
   0 0 0 0 0 0];

B3=[0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];

C3=[1 0 0 0 1 0;
    0 0 1 0 0 1
    1 0 0 0 0 0
    0 0 1 0 0 0];

A4=[0 1 0 0 0 0 0;
   0 0 0 0 0 0 0;
   0 0 0 1 0 0 0;
   0 0 0 0 0 0 0;
   0 0 0 0 0 0 0;
   0 0 0 0 0 0 0
   0 0 0 0 0 0 0];

B4=[0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];

C4=[1 0 0 0 1 0 0;
    0 0 1 0 0 1 0;
    1 0 0 0 0 0 0;
    0 0 1 0 0 0 0;
    0 0 0 0 0 0 1];

%%
% A = [wy*cos(fi)*tan(theta) - wz*sin(fi)*tan(theta),                       1, wz*cos(fi)*(tan(theta)^2 + 1) + wy*sin(fi)*(tan(theta)^2 + 1),      sin(fi)*tan(theta), 0, 0,      cos(fi)*tan(theta);
%                                             0,                       0,                                                             0, (Jy*wz)/Jx - (Jz*wz)/Jx, 0, 0, (Jy*wy)/Jx - (Jz*wy)/Jx;
%                    - wz*cos(fi) - wy*sin(fi),                       0,                                                             0,                 cos(fi), 0, 0,                -sin(fi);
%                                             0, (Jz*wz)/Jy - (Jx*wz)/Jy,                                                             0,                       0, 0, 0, (Jz*wx)/Jy - (Jx*wx)/Jy;
%                                             0,                       0,                                                             0,                       0, 0, 0,                       0;
%                                             0,                       0,                                                             0,                       0, 0, 0,                       0;
%                                            0, (Jx*wy)/Jz - (Jy*wy)/Jz,                                                             0, (Jx*wx)/Jz - (Jy*wx)/Jz, 0, 0,                       0]
%  
J=[1.395*10^-5 0 0
    0 1.436*10^-5 0
    0 0 2.173*10^-5];
Jx=J(1,1);
Jy=J(2,2);
Jz=J(3,3);
 
% Bw =[1,    0, 0,    0, 0, 0,    0;
% 	0, 1/Jx, 0,    0, 0, 0,    0;
% 	0,    0, 1,    0, 0, 0,    0;
% 	0,    0, 0, 1/Jy, 0, 0,    0;
% 	0,    0, 0,    0, 1, 0,    0;
% 	0,    0, 0,    0, 0, 1,    0;
% 	0,    0, 0,    0, 0, 0, 1/Jz];
%%
%G = [1;0;0;0];
%H = C2;
%%ny = size(A1,1);
%%nx = size(C1,2);

ny1 = 2;
nx1 = 6;

ny2 = 2;
nx2 = 4;

nx3=6;
ny3=4;

nx4=7;
ny4=5;

%A = A4;
B = B4;
C = C4;

nx = nx4;
ny = ny4;

Ts = t(2)-t(1);
%A_discrete = expm(A*Ts); % discrete state matrix

% kalman gains
Q =(Ts^2)*eye(nx);%Ts^2*10*eye(nx); %vamos assumir este no nosso, isto é op ruido de preocessamento mas depois falamos com o prof a perguntar se vale a pena mudar a covariancia (por exemplo o vento mudou a velocidade, no caso de um carro a estrada pode ter uns altinhos...)
Q(1,1)=Q(1,1)*1;
Q(2,2)=Q(2,2)*100;
Q(3,3)=Q(3,3)*1;
Q(4,4)=Q(4,4)*100;
Q(5,5)=Q(5,5)*0.000001;
Q(6,6)=Q(6,6)*0.000001;
Q(7,7)=Q(7,7)*100;

R = (Ts^2)*eye(ny); %isto é o ruido dos sensores e covariancia dos outputs , posso ter trocado com o de cima (por exemplo resolução do sensor, ruido proveniente de campos magneticos...)
%R(1,1)=0.0151; %R(1,1)*roll_cov;
%R(2,2)=0.0256; %R(2,2)*theta_cov;

R(1,1) =R(1,1)*1.2509e-06;%6.0931e-05;0.7679
R(2,2) =R(2,2)*1.3762e-06;%8.051e-07;0.5938
R(3,3) =R(3,3)*2.0212e-06;%0.8898%2.7225e-04
R(4,4) =R(4,4)*8.4940e-06;%0.6680%2.0449e-04
R(5,5) =R(4,4)*1.4743e-07;

%rk_ctrb = rank(ctrb(A,Q))
%rk_obsb = rank(obsv(A,C))

% initial values
P0 = 2*eye(nx);

P0=Q;

P0(1,1)=Q(1,1)*100;
P0(2,2)=Q(2,2)*0.01;
P0(3,3)=Q(3,3)*100;
P0(4,4)=Q(4,4)*0.001;
P0(5,5)=Q(5,5)*0.0001;
P0(6,6)=Q(6,6)*0.0001;
P0(7,7)=Q(7,7)*0.01;


x01 = [roll(1); gyro(1,1)*pi/180; pitch(1); gyro(2,1)*pi/180; 0.1; 0.1];
x02 = [roll(1); gyro(1,1)*pi/180; pitch(1); gyro(2,1)*pi/180];
x04 = [roll(1); gyro(1,1)*pi/180; pitch(1); gyro(2,1)*pi/180; 0.1; 0.1; gyro(3,1)];
Bw=eye(7);
Dv=eye(5);

w_mean=[0;0;0;0;0;0];

%Preguntar ao stor sobre a covariancia do processamento?; o que considerar

% initial values
% simulate system and estimation
%Tsim = t(length(t));
%t = 0:Ts:Tsim;
Nsim = length(t);
%u_th = Ts*0.2*sin(2*pi/4*t);
x = zeros(nx,Nsim); %vetor dos estados medidos ao longo do tempo
y = zeros(ny,Nsim); %vetor dos outputs do proprio drone ao longo do tempo
y = [roll; pitch; roll_gyros; pitch_gyros; gyro(3,:)];
xe = zeros(nx,Nsim);
Pe = zeros(nx,nx,Nsim);
%x(:,1) = x02;
xe(:,1) = x04; %estado inicial estimado
Pe(:,:,1) = P0;%covariancia inicial estimada
for k = 1:Nsim
   wy=xe(4,k);
   wx=xe(2,k); 
   wz=xe(7,k);
   fi=xe(1,k);
   theta=xe(3,k);
    A = [wy*cos(fi)*tan(theta) - wz*sin(fi)*tan(theta),                       1, wz*cos(fi)*(tan(theta)^2 + 1) + wy*sin(fi)*(tan(theta)^2 + 1),      sin(fi)*tan(theta), 0, 0,      cos(fi)*tan(theta);
                                            0,                       0,                                                             0, (Jy*wz)/Jx - (Jz*wz)/Jx, 0, 0, (Jy*wy)/Jx - (Jz*wy)/Jx;
                   - wz*cos(fi) - wy*sin(fi),                       0,                                                             0,                 cos(fi), 0, 0,                -sin(fi);
                                            0, (Jz*wz)/Jy - (Jx*wz)/Jy,                                                             0,                       0, 0, 0, (Jz*wx)/Jy - (Jx*wx)/Jy;
                                            0,                       0,                                                             0,                       0, 0, 0,                       0;
                                            0,                       0,                                                             0,                       0, 0, 0,                       0;
                                           0, (Jx*wy)/Jz - (Jy*wy)/Jz,                                                             0, (Jx*wx)/Jz - (Jy*wx)/Jz, 0, 0,                       0];
     A_discrete = expm(A*Ts);
    
    % predict next estimate:
    %Bu = B*u;
    Bu = zeros(nx,1); % não se tem inputs associados ao roll e pitch logo B é uma matriz de 0s
    [xem,Pem] = extended_kalman_predict(x(:,k),Pe(:,:,k),A_discrete,Bu,Q,Bw,w_mean); %devolve o estado e covariancia estimda do proximo instante  sabendo o instante anterior
    %o G é o B e o F o A
    %o Q é o erro de processamento e covariância do sensores ou y



    % update estimate with measurement info
    [xe(:,k),Pe(:,:,k),K] = extended_kalman_update(xem,Pem,y(:,k),C,R,Dv); %atualiza os estados com base nos outupts previstos e medidos pelos sensores, e retorna o ganho de kalman
    %o xem e Pem digo la em cima, o y a saida dos sensores, H o C e R o
    %ruido dos sensores



end 


% Show results
figure(20051);
plot(t,(xe(1,:)*180/pi),t,lbd(1,:));%+xe(5,:)
grid on;
xlabel('t [s]');
ylabel('$$\phi$$ [deg]');
legend('est','real');
title('roll angle estimates')



figure(20052);
plot(t,(xe(3,:)*180/pi),t,lbd(2,:));%+xe(6,:)
grid on;
xlabel('t [s]');
ylabel('$$\theta$$ [deg]');
legend('est','real');
title('pitch angle estimates')

media3=xe(1,:)*180/pi-lbd(1,:);
media3= mean_function(media3)
media4=xe(3,:)*180/pi-lbd(2,:);
media4= mean_function(media4)







