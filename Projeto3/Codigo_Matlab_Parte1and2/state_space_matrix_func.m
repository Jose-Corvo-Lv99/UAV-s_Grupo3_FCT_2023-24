function [A,B] = state_space_matrix_func (fi, theta, yaw)


%1.4.
A =[0, 0, 0, cos(theta)*cos(yaw), sin(fi)*cos(yaw)*sin(theta) - cos(fi)*sin(yaw), sin(fi)*sin(yaw) + cos(fi)*cos(yaw)*sin(theta)
    0, 0, 0, cos(theta)*sin(yaw), cos(fi)*cos(yaw) + sin(fi)*sin(theta)*sin(yaw), cos(fi)*sin(theta)*sin(yaw) - sin(fi)*cos(yaw)
    0, 0, 0,         -sin(theta),                             cos(theta)*sin(fi),                             cos(fi)*cos(theta)
    0, 0, 0,             dragx/m,                                              0,                                            -wy
    0, 0, 0,                   0,                                        dragy/m,                                             wx
    0, 0, 0,                  wy,                                            -wx,                                        dragz/m];


B =[  0,   vy*(sin(fi)*sin(yaw) + cos(fi)*cos(yaw)*sin(theta)) + vz*(cos(fi)*sin(yaw) - sin(fi)*cos(yaw)*sin(theta)), vz*cos(fi)*cos(theta)*cos(yaw) - vx*cos(yaw)*sin(theta) + vy*cos(theta)*sin(fi)*cos(yaw), vz*(sin(fi)*cos(yaw) - cos(fi)*sin(theta)*sin(yaw)) - vy*(cos(fi)*cos(yaw) + sin(fi)*sin(theta)*sin(yaw)) - vx*cos(theta)*sin(yaw)
      0, - vy*(sin(fi)*cos(yaw) - cos(fi)*sin(theta)*sin(yaw)) - vz*(cos(fi)*cos(yaw) + sin(fi)*sin(theta)*sin(yaw)), vz*cos(fi)*cos(theta)*sin(yaw) - vx*sin(theta)*sin(yaw) + vy*cos(theta)*sin(fi)*sin(yaw), vz*(sin(fi)*sin(yaw) + cos(fi)*cos(yaw)*sin(theta)) - vy*(cos(fi)*sin(yaw) - sin(fi)*cos(yaw)*sin(theta)) + vx*cos(theta)*cos(yaw)
      0,                                                               vy*cos(fi)*cos(theta) - vz*cos(theta)*sin(fi),                          - vx*cos(theta) - vz*cos(fi)*sin(theta) - vy*sin(fi)*sin(theta),                                                                                                                                  0
      0,                                                                                                           0,                                                                 g_earth*cos(conj(theta)),                                                                                                                                  0
      0,                                                                     -g_earth*cos(conj(fi))*cos(conj(theta)),                                                   g_earth*sin(conj(fi))*sin(conj(theta)),                                                                                                                                  0
    1/m,                                                                      g_earth*cos(conj(theta))*sin(conj(fi)),                                                   g_earth*cos(conj(fi))*sin(conj(theta)),                                                                                                                                  0];





Q=eye(6);
    
R=eye(4);
end
