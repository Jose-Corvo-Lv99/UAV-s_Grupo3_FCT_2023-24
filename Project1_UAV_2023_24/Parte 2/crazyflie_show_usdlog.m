% crazyflie load usd card log csv file

% read file
csvfilename = '2024-04-04_log07.csv';
T = readtable(csvfilename);

% get data from table
time = table2array(T(:,1))'*1e-3;
pos = table2array(T(:,2:4))';
vel = table2array(T(:,5:7))';
% acc = table2array(T(:,8:10))';
lbd = table2array(T(:,8:10))'*pi/180;
om = table2array(T(:,11:13))'*pi/180;
pos_ref = table2array(T(:,14:16))';
yaw_ref = table2array(T(:,17))';
motors = table2array(T(:,18:21))';

% convert date to print format
t = time - time(1);
x = [pos;vel;lbd;om];
x_ref = [pos_ref;0*vel;lbd*0;om*0];
x_ref(9,:) = yaw_ref;
u = motors/65535.0;
Ts=t(2)-t(1);

pos_mov_z=pos(:,8643:18302); T_mov_z=u(1,8643:18302)+u(2,8643:18302)+u(3,8643:18302)+u(4,8643:18302);
pos_mov_x=pos(:,18303:26182);T_mov_x=u(1,18303:26182)+u(2,18303:26182)+u(3,18303:26182)+u(4,18303:26182);
pos_mov_y=pos(:,26182:length(pos));T_mov_y=u(1,26182:length(pos))+u(2,26182:length(pos))+u(3,26182:length(pos))+u(4,26182:length(pos));
%pos_mov_z4=pos(:,8643:18302); T_mov_z4=u(1,8643:18302)+u(2,8643:18302)+u(3,8643:18302)+u(4,8643:18302);
%pos_mov_z3=pos(:,9532:18302); T_mov_z3=u(1,9532:18302)+u(2,9532:18302)+u(3,9532:18302)+u(4,9532:18302);
%pos_mov_z34=pos(:,9532:13650); T_mov_z34=u(1,9532:13650)+u(2,9532:13650)+u(3,9532:13650)+u(4,9532:13650);
%pos_mov_z2=pos(:,13650:18302); T_mov_z2=u(1,13650:18302)+u(2,13650:18302)+u(3,13650:18302)+u(4,13650:18302);
%pos_mov_z=pos(:,13650:15938); T_mov_z=u(1,13650:15938)+u(2,13650:15938)+u(3,13650:15938)+u(4,13650:15938);
%pos_mov_x=pos(:,18303:26182);T_mov_x=u(1,18303:26182)+u(2,18303:26182)+u(3,18303:26182)+u(4,18303:26182);
%pos_mov_y=pos(:,26182:length(pos));T_mov_y=u(1,26182:length(pos))+u(2,26182:length(pos))+u(3,26182:length(pos))+u(4,26182:length(pos));
fi_mov_z=lbd(1,8643:18302);
fi_mov_x=lbd(1,18303:26182); 
fi_mov_y=lbd(1,26182:length(lbd)); 
theta_mov_z=lbd(2,8643:18302);
theta_mov_x=lbd(2,18303:26182); 
theta_mov_y=lbd(2,26182:length(lbd));
%pos_mov_z9=pos(:,:); T_mov_z9=u(1,:)+u(2,:)+u(3,:)+u(4,:);

% plot data
initPlots;
vehicle3d_ref_show_data(t,x,u,x_ref);