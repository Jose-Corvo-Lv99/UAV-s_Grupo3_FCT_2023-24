%Sistem eq Solver

syms vx_eq_inercial vy_eq_inercial T 
dragx=-0.001040535000000;
dragy=-0.001213957500000;
dragz=-0.001916775000000;
g_earth=9.82%m/s
m=0.035%kg
yaw_in=0;
theta_in=pi/360;
fi_in=pi/360;

eqn1= [g_earth*sin(conj(theta_in)) - (dragx*(vy_eq_inercial*(cos(fi_in)*sin(yaw_in) - sin(fi_in)*cos(yaw_in)*sin(theta_in)) - vx_eq_inercial*cos(theta_in)*cos(yaw_in)))/m==0, (dragy*(vy_eq_inercial*(cos(fi_in)*cos(yaw_in) + sin(fi_in)*sin(theta_in)*sin(yaw_in)) + vx_eq_inercial*cos(theta_in)*sin(yaw_in)))/m - g_earth*cos(conj(theta_in))*sin(conj(fi_in))==0, (T - dragz*(vx_eq_inercial*sin(theta_in) - vy_eq_inercial*cos(theta_in)*sin(fi_in)))/m - g_earth*cos(conj(fi_in))*cos(conj(theta_in))==0]

%%eqn2= (dragy*(vy_eq_inercial*(cos(fi_in)*cos(yaw_in) + sin(fi_in)*sin(theta_in)*sin(yaw_in)) + vx_eq_inercial*cos(theta_in)*sin(yaw_in)))/m - g_earth*cos(conj(theta_in))*sin(conj(fi_in)) == 0;
%%S = solve (eqn2);
%%vyyy = vpa(S)


S = vpasolve (eqn1, [vy_eq_inercial vx_eq_inercial T])
%vxxxx = vpa(S)


veqqq = Euler2R([fi_in;theta_in;yaw_in])'*[2.8827672712878218444836116856714;-2.4706880189719336114995718566559;0]
