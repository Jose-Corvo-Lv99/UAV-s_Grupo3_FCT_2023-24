function [A, B, C, D] = SS_matrices(px, py, pz, fi, theta, yaw, vx, vy, vz, wx, wy, wz, g_earth, m, Jx, Jy, Jz, dragx, dragy, dragz)

%matrix A
A =[0, 0, 0,   vy*(sin(fi)*sin(yaw) + cos(fi)*cos(yaw)*sin(theta)) + vz*(cos(fi)*sin(yaw) - sin(fi)*cos(yaw)*sin(theta)), vz*cos(fi)*cos(theta)*cos(yaw) - vx*cos(yaw)*sin(theta) + vy*cos(theta)*sin(fi)*cos(yaw), vz*(sin(fi)*cos(yaw) - cos(fi)*sin(theta)*sin(yaw)) - vy*(cos(fi)*cos(yaw) + sin(fi)*sin(theta)*sin(yaw)) - vx*cos(theta)*sin(yaw), cos(theta)*cos(yaw), sin(fi)*cos(yaw)*sin(theta) - cos(fi)*sin(yaw), sin(fi)*sin(yaw) + cos(fi)*cos(yaw)*sin(theta),                       0,                       0,                       0
    0, 0, 0, - vy*(sin(fi)*cos(yaw) - cos(fi)*sin(theta)*sin(yaw)) - vz*(cos(fi)*cos(yaw) + sin(fi)*sin(theta)*sin(yaw)), vz*cos(fi)*cos(theta)*sin(yaw) - vx*sin(theta)*sin(yaw) + vy*cos(theta)*sin(fi)*sin(yaw), vz*(sin(fi)*sin(yaw) + cos(fi)*cos(yaw)*sin(theta)) - vy*(cos(fi)*sin(yaw) - sin(fi)*cos(yaw)*sin(theta)) + vx*cos(theta)*cos(yaw), cos(theta)*sin(yaw), cos(fi)*cos(yaw) + sin(fi)*sin(theta)*sin(yaw), cos(fi)*sin(theta)*sin(yaw) - sin(fi)*cos(yaw),                       0,                       0,                       0
    0, 0, 0,                                                               vy*cos(fi)*cos(theta) - vz*cos(theta)*sin(fi),                          - vx*cos(theta) - vz*cos(fi)*sin(theta) - vy*sin(fi)*sin(theta),                                                                                                                                  0,         -sin(theta),                             cos(theta)*sin(fi),                             cos(fi)*cos(theta),                       0,                       0,                       0
    0, 0, 0,                                                               wy*cos(fi)*tan(theta) - wz*sin(fi)*tan(theta),                            wz*cos(fi)*(tan(theta)^2 + 1) + wy*sin(fi)*(tan(theta)^2 + 1),                                                                                                                                  0,                   0,                                              0,                                              0,                       1,      sin(fi)*tan(theta),      cos(fi)*tan(theta)
    0, 0, 0,                                                                                   - wz*cos(fi) - wy*sin(fi),                                                                                        0,                                                                                                                                  0,                   0,                                              0,                                              0,                       0,                 cos(fi),                -sin(fi)
    0, 0, 0,                                                           (wy*cos(fi))/cos(theta) - (wz*sin(fi))/cos(theta),              (wz*cos(fi)*sin(theta))/cos(theta)^2 + (wy*sin(fi)*sin(theta))/cos(theta)^2,                                                                                                                                  0,                   0,                                              0,                                              0,                       0,      sin(fi)/cos(theta),      cos(fi)/cos(theta)
    0, 0, 0,                                                                                                           0,                                                                 g_earth*cos(conj(theta)),                                                                                                                                  0,             dragx/m,                                             wz,                                            -wy,                       0,                     -vz,                      vy
    0, 0, 0,                                                                     -g_earth*cos(conj(fi))*cos(conj(theta)),                                                   g_earth*sin(conj(fi))*sin(conj(theta)),                                                                                                                                  0,                 -wz,                                        dragy/m,                                             wx,                      vz,                       0,                     -vx
    0, 0, 0,                                                                      g_earth*cos(conj(theta))*sin(conj(fi)),                                                   g_earth*cos(conj(fi))*sin(conj(theta)),                                                                                                                                  0,                  wy,                                            -wx,                                        dragz/m,                     -vy,                      vx,                       0
    0, 0, 0,                                                                                                           0,                                                                                        0,                                                                                                                                  0,                   0,                                              0,                                              0,                       0, (Jy*wz)/Jx - (Jz*wz)/Jx, (Jy*wy)/Jx - (Jz*wy)/Jx
    0, 0, 0,                                                                                                           0,                                                                                        0,                                                                                                                                  0,                   0,                                              0,                                              0, (Jz*wz)/Jy - (Jx*wz)/Jy,                       0, (Jz*wx)/Jy - (Jx*wx)/Jy
    0, 0, 0,                                                                                                           0,                                                                                        0,                                                                                                                                  0,                   0,                                              0,                                              0, (Jx*wy)/Jz - (Jy*wy)/Jz, (Jx*wx)/Jz - (Jy*wx)/Jz,                       0];



%matrix B
B =[  0,    0,    0,    0
      0,    0,    0,    0
      0,    0,    0,    0
      0,    0,    0,    0
      0,    0,    0,    0
      0,    0,    0,    0
      0,    0,    0,    0
      0,    0,    0,    0
    1/m,    0,    0,    0
      0, 1/Jx,    0,    0
      0,    0, 1/Jy,    0
      0,    0,    0, 1/Jz];

%C = [eye(3), zeros(3,9); zeros(1,5),1, zeros(1,6)];
C=eye(12);

D = zeros(12,4);
%D = zeros(4,4);

end