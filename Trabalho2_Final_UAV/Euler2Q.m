function Q = Euler2Q(l)

phi = l(1); theta = l(2);
Q = [  1 sin(phi)*tan(theta) cos(phi)*tan(theta);
       0 cos(phi)           -sin(phi);
       0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
