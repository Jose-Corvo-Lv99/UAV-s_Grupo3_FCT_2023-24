syms omgx omgy omgz roll pitch theta yaw Jx Jy Jz 


lbd = [roll; pitch; yaw];
omg = [omgx; omgy; omgz];

J=[Jx 0 0;
   0 Jy 0;
    0 0 Jz];

lbd_dot1 = Euler2Q(lbd)*omg


om_dot = -inv(J)*skew(omg)*J*omg


