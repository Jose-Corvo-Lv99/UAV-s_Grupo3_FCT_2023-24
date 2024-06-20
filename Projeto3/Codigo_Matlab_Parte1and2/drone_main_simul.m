% NOVA School of Science and Technology
% Department of Electrical and Computer Engineering
% 2022
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: simulate simple dynamic system model of a drone

clear all;

drone_init;

% initialize variables for all drones:
t = 0:dTi:Tend;
nt = length(t);
nx = 18;
nu = 4;
for iD = 1:nD
    p0{iD} = [0;0.2*((nD-1)/2-iD+1);0];
    v0{iD} = [0;0;0];
    R0{iD} = Euler2R([0;0;pi/20*((nD-1)/2-iD+1)]);
    om0{iD} = [0;0;0];
    x{iD} = zeros(nx,Nsim+1);
    xiep{iD} = zeros(3,Nsim+1);
    T{iD} = zeros(1,Nsim);
    tau{iD} = zeros(3,Nsim);
    lbd{iD} = zeros(3,Nsim);
    x{iD}(:,1) = [p0{iD};v0{iD};reshape(R0{iD},[],1);om0{iD}];
    
    if ref_mode == 1 % step reference
        p_ref{iD} = [ zeros(3,20/Param.dTi+1), p_ref_static*ones(1,(Tend-20)/Param.dTi)];
        v_ref{iD} = zeros(3,nt);
        a_ref{iD} = zeros(3,nt);
        j_ref{iD} = zeros(3,nt);
        psi_ref{iD} = [ zeros(1,20/Param.dTi+1), psi_ref_static*ones(1,(Tend-20)/Param.dTi)];
        dpsi_ref{iD} = [ zeros(1,20/Param.dTi+1), zeros(1,(Tend-20)/Param.dTi)];
        
    else % circle reference
        phase{iD} = (iD-1)*dphase;
        p_ref{iD} = [Rad*cos(omn*t+phase{iD});Rad*sin(omn*t+phase{iD});(1+dh*(iD-1))*ones(size(t))];
        v_ref{iD} = [-Rad*omn*sin(omn*t+phase{iD});Rad*omn*cos(omn*t+phase{iD});0*ones(size(t))];
        a_ref{iD} = [-Rad*omn^2*cos(omn*t+phase{iD});-Rad*omn^2*sin(omn*t+phase{iD});0*ones(size(t))];
        j_ref{iD} = [ Rad*omn^3*sin(omn*t+phase{iD});-Rad*omn^3*cos(omn*t+phase{iD});0*ones(size(t))];
        dp_ref = p_ref{iD}(:,2:end) - p_ref{iD}(:,1:end-1);
        dp_ref = [ dp_ref , dp_ref(:,end) ];
        psi_ref{iD} = atan2(dp_ref(2,:),dp_ref(1,:));
        dpsi_ref{iD} = median(psi_ref{iD}(:,2:end) - psi_ref{iD}(:,1:end-1))*ones(size(psi_ref{iD}));
    end
end

% main time loop for simulation
for k = 1:Nsim
    for iD = 1:nD
                
        % get state vector and plot it
        p{iD} = x{iD}(1:3,k);
        v{iD} = x{iD}(4:6,k);
        R = reshape(x{iD}(7:15,k),3,3);
        om = x{iD}(16:18,k);
        p_d{iD} = p_ref{iD}(:,k);
        v_d{iD} = v_ref{iD}(:,k);
        a_d = a_ref{iD}(:,k);
        j_d = j_ref{iD}(:,k);
        psi_d = psi_ref{iD}(:,k);
        dpsi_d = dpsi_ref{iD}(:,k);
        
        % outer-loop controller
        e_p = p{iD} - p_d{iD};
        e_v = v{iD} - v_d{iD};
        if mod(k*dTi,dTo) == 0 || k == 1
            [Tdes{iD},Rdes{iD}] = drone_outer_ctrl(e_p,e_v,R,psi_d,Param,xiep{iD}(:,k),a_d);
        end
        T{iD}(:,k) = Tdes{iD};
        xiep{iD}(:,k+1) = xiep{iD}(:,k) + Param.dTi*e_p;
            
        % nonlinear inner-loop controller
        tau{iD}(:,k) = drone_att_ctrl(R,Rdes{iD},om,T{iD}(:,k),Param,j_d,dpsi_d);
        
        % nonlinear drone model
        [dot_p,dot_v,dot_R,dot_om] = drone_3dfull_dyn(v{iD},R,om,T{iD}(:,k),tau{iD}(:,k),Param);
        
        % discretization 
        pp = p{iD} + Param.dTi*dot_p;
        vp = v{iD} + Param.dTi*dot_v;
        Rp = rot_integrate(R,om,Param.dTi);
        if abs(1-norm(Rp'*Rp))>1e-4, error('Problems with rotation matrixx integration.'); end
        omp = om + Param.dTi*dot_om;
        x{iD}(:,k+1) = [pp;vp;reshape(Rp,[],1);omp];
        
        % auxiliary drone attitude computation from rotation matrix
        lbd{iD}(:,k) = R2Euler(R);
        
    end
end

p_qua_sum = 0;
v_qua_sum = 0;
for i = 1:Nsim
    p_dif = x{1}(1:3,i+1) - p_ref{1}(:,i);
    p_dif = p_dif.^2;
    p_qua_sum =p_qua_sum + (p_dif(1)+ p_dif(2)+ p_dif(3))^0.5;

    v_dif = x{1}(4:6,i+1) - v_ref{1}(:,i);
    v_dif = v_dif.^2;
    v_qua_sum =v_qua_sum + (v_dif(1)+ v_dif(2)+ v_dif(3))^0.5;
end

prmse = (p_qua_sum/Nsim)^0.5

vrmse = (v_qua_sum/Nsim)^0.5

% trim vectors to same length
clear p;
for iD = 1:nD
    x{iD}(:,k+1) = [];
    p{iD} = x{iD}(1:3,:);
end

drone_show_data;
drone_animate(p,p_ref,lbd,t,dcolors);





