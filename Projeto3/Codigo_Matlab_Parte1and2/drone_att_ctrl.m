function tau = drone_att_ctrl(R,R_des,om,T,P,j_ref,dpsi_ref)
%DRONE_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

    if ~exist('dpsi_ref','var') || isempty(dpsi_ref), dpsi_ref = 0; end
    if ~exist('j_ref','var') || isempty(j_ref), j_ref = zeros(3,1); end
    
    % define errors and auxiliary variables:
    zW = [0;0;1];
    xB_des = R_des(:,1);
    yB_des = R_des(:,2);
    zB_des = R_des(:,3);
    
    % compute torques
    hw_des = P.m/T*(j_ref - (zB_des'*j_ref)*zB_des);
    om_des = [-hw_des'*yB_des
               hw_des'*xB_des
               dpsi_ref*zW'*zB_des ];
    eom = om-om_des;
    eR = 1/2*unskew(R_des'*R - R'*R_des);
    tau = -P.kR*eR - P.kom*eom;
    
end

