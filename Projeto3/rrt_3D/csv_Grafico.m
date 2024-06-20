clc
clear all
x='dados_recebidos_de0paraGoal1.csv'

% read file
csvfilename = x;
array = dlmread(csvfilename,',',1,0);

px=array(:,1);
py=array(:,2);
pz=array(:,3);
t=1:length(array(:,1));

figure(10);
plot([px,py,pz],t);
hold on;
grid on;
xlabel('p(t) [m]');
title('Variação do p');

figure(20);
plot(px,t);
hold on;
grid on;
xlabel('px(t) [m]');
title('Variação do px');

figure(21);
plot(py,t);
hold on;
grid on;
xlabel('py(t) [m]');
title('Variação do py');

figure(22);
plot(pz,t);
hold on;
grid on;
xlabel('pz(t) [m]');
title('Variação do pz');

