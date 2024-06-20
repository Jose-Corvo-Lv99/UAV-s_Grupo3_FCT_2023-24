function [T, R_des] = drone_outer_ctrl(e_p,e_v,R, psi_ref,P,ie_p, a_ref)
%DRONE_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

    if ~exist('psi_ref','var') || isempty(psi_ref), psi_ref = 0; end
    if ~exist('ie_p','var') || isempty(ie_p), ie_p = zeros(3,1); end
    if ~exist('a_ref','var') || isempty(a_ref), a_ref = zeros(3,1); end
    
    % define errors and auxiliary variables:
    zW = [0;0;1];
    zB = R(:,3);
    
    % Mellinger Controller (up to attitude commands)
    f_des = -P.kp*e_p - P.kv*e_v + P.m*P.g*zW + P.m*a_ref;

    % compute thrust
    T = f_des'*zB; 

    % compute desired rotation matrix
    zB_des = f_des/norm(f_des);
     xC_des = [cos(psi_ref);sin(psi_ref);0];
     yB_des = skew(zB_des)*xC_des/norm(skew(zB_des)*xC_des);
     xB_des = skew(yB_des)*zB_des;
     R_des = [xB_des,yB_des,zB_des];
    
end

