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
csvfilename = csvfilename_motors_off;
array = dlmread(csvfilename,',',1,0);
%T = table2array(readtable(csvfilename)); % Matlab only

% get data from table
time = array(:,1)'*1e-3;
pos = array(:,2:4)'; % [m] groundtruth
vel = array(:,5:7)'; % [m/s] groundtruth
lbd = array(:,8:10)'; % [deg] groundtruth
gyro = array(:,11:13)'; % [deg/s] sensor
acc = array(:,14:16)'; % [Gs] sensor
baro_asl = array(:,17)'; % [m] sensor
% lighthouse = array(:,18:20))'; % [m]

% convert date to print format
t = time - time(1);

% plot data
initPlots;
crazyflie_show_sensor_data(t,acc,gyro,baro_asl,pos,vel,lbd);

%%Calculo das medias e covariancias
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

avg_matrix=[posX_avg, velX_avg, roll_avg, omgX_avg, accX_avg, 0;
            posY_avg, velY_avg, theta_avg, omgY_avg, accY_avg, 0;
            posZ_avg, velZ_avg, yaw_avg, omgZ_avg, accZ_avg, baro_asl_avg]



cov_matrix=[posX_cov, velX_cov, roll_cov, omgX_cov, accX_cov, 0;
            posY_cov, velY_cov, theta_cov, omgY_cov, accY_cov, 0;
            posZ_cov, velZ_cov, yaw_cov, omgZ_cov, accZ_cov, baro_asl_cov]


g = 1;
ax = acc(1,:);
ay = acc(2,:);
az = acc(3,:);




%roll = atan2(ay, az);   %rad
%pitch = asin(ax, g);    %rad

roll = arrayfun( @(x,y) atan2(x,y), ay, az); %rad
pitch = arrayfun( @(x) asin(x), ax.*((g*ones(1, length(ax))).^-1) ); %rad

covariance_function(roll,roll)
covariance_function(pitch,pitch)
%figure(20);
%subplot(2,1,1);
%plot(t,roll);

%subplot(2,1,2);
%plot(t,pitch);

figure(20);
subplot(2,1,1);
plot(t,roll, t, lbd(1,:)*(pi/180)); %assim é como faz mais sentido com isto ao quadrado
grid on;
xlabel('t [s]');
ylabel('$$\phi$$ [rad]');
legend('sensors','given');
title('roll angle estimates')



subplot(2,1,2);
plot(t,pitch, t, lbd(2,:)*(pi/180));
grid on;
xlabel('t [s]');
ylabel('$$\theta$$ [rad]');
legend('sensors','given');
title('pitch angle estimates')




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

figure(30);
subplot(2,1,1);
plot(t,roll_gyros*pi/180, t, lbd(1,:)*pi/180); %assim é como faz mais sentido com isto ao quadrado
grid on;
xlabel('t [s]');
ylabel('$$\phi$$ [rad]');
legend('sensors','given');
title('roll angle estimates')

subplot(2,1,2);
plot(t,pitch_gyros*pi/180, t, lbd(2,:)*pi/180);
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

