function [x,P,K] = kalman_update(x,P,y,H,R)
% KALMAN_UDATE General discrete time-varying kalman filter update step.
% Author: BGuerreiro 2011
    
    % Compute Kalman optimal gain
    % See [Gelb - APPLIED OPTIMAL ESTIMATION] Table 4.2-1 (normal KF) and Table 6.1-1 (Extended KF)
    K = P*H'/(H*P*H' + R);

    % Update State estimate
    x = x + K*(y - H*x);
    
    % Update covariance matrix
    P = P - K*H*P;
    % Projection over Sn subspace (keep Pk symmetric)
    P = (P+P')/2;

end