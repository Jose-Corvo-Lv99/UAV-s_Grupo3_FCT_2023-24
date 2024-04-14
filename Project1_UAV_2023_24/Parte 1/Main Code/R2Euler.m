% NOVA School of Science and Technology
% Department of Electrical and Computer Engineering
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
function lbd = R2Euler(R)
%   lbd = R2euler(R) = [phi;theta;psi]  ,  with  R in SO(3)
%   lbd in R^3 is the vector of the Z-Y-X euler angles that define R
%
    lbd = zeros(3,1);
    lbd(2,1) = atan2(-R(3,1),sqrt(R(1,1)^2 + R(2,1)^2));
    if abs(cos(lbd(2,1))) < eps %if it is a singular configuration
        lbd(3,1) = 0; %Yaw
        lbd(1,1) = sign(lbd(2,1))*atan2(R(1,2),R(2,2)); %Roll
    else
        lbd(3,1) = atan2(R(2,1)/cos(lbd(2)),R(1,1)/cos(lbd(2))); %Yaw
        lbd(1,1) = atan2(R(3,2)/cos(lbd(2)),R(3,3)/cos(lbd(2))); %Roll
    end
    
end