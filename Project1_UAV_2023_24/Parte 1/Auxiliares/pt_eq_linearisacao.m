pkg load symbolic
syms px_in py_in pz_in fi_in theta_in yaw_in fi_eq theta_eq yaw_eq vx_eq vy_eq vz_eq wx_eq wy_eq wz_eq T1_eq T2_eq T3_eq T4_eq dragx dragy dragz g_earth m vx_eq_inercial vy_eq_inercial;



lbd_in = [fi_in
       theta_in
       yaw_in];

v = [vx_eq
     vy_eq
     vz_eq];

lbd = [fi_eq
       theta_eq
       yaw_eq];

omg = [wx_eq
       wy_eq
       wz_eq];


R = Euler2R(lbd);
R_in = Euler2R(lbd_in);

zI=[0; 0; 1];
%Horizontal Flight
fp=[0;0;T1_eq];
v_inercial=[vx_eq_inercial; vy_eq_inercial;0];
v=R_in*v_inercial;
Drag_Body=diag([dragx,dragy,dragz]);
fa=Drag_Body*v;
v_dot = - g_earth*R_in'*zI+(1/m)*(fa+fp)


%g_earth*sin(conj(theta_in)) - (dragx*(vy_eq_inercial*(cos(fi_in)*sin(yaw_in) - sin(fi_in)*cos(yaw_in)*sin(theta_in)) - vx_eq_inercial*cos(theta_in)*cos(yaw_in)))/m
%(dragy*(vy_eq_inercial*(cos(fi_in)*cos(yaw_in) + sin(fi_in)*sin(theta_in)*sin(yaw_in)) + vx_eq_inercial*cos(theta_in)*sin(yaw_in)))/m - g_earth*cos(conj(theta_in))*sin(conj(fi_in))
%                                           (T1_eq - dragz*(vx_eq_inercial*sin(theta_in) - vy_eq_inercial*cos(theta_in)*sin(fi_in)))/m - g_earth*cos(conj(fi_in))*cos(conj(theta_in))

