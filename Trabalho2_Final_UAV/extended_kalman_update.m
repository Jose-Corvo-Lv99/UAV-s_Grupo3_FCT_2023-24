function [x,P,K] = extended_kalman_update(x,P,y,C,R, Dv)

    
    % Compute Kalman optimal gain
    % See [Gelb - APPLIED OPTIMAL ESTIMATION] Table 4.2-1 (normal KF) and Table 6.1-1 (Extended KF)
    K = P*C'/(C*P*C' + Dv*R*Dv');

    % Update State estimate
    x = x + K*(y - C*x);
    
    % Update covariance matrix
    P = P-K*C*P;
    % Projection over Sn subspace (keep Pk symmetric)
    P = (P+P')/2;

end