function [x,P] = kalman_predict(x,P,F,Gu,Q)
% KALMAN_UDATE General discrete time-varying kalman filter update step.
% Author: BGuerreiro 2011

    % Project covariance matrix ahead
    P = F*P*F' + Q;
    % Projection over Sn subspace (keep Pk symmetric)
    P = (P+P')/2;
    
    % State prediction
    x = F*x + Gu;

end