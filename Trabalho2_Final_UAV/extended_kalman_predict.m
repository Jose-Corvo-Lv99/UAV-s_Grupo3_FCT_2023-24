function [x,P] = extended_kalman_predict(x,P,A,Bu,Q,Bw,w_mean)
% KALMAN_UDATE General discrete time-varying kalman filter update step.
% Author: BGuerreiro 2011

    % Project covariance matrix ahead
    P = A*P*A' + Bw*Q*Bw';
    % Projection over Sn subspace (keep Pk symmetric)
    P = (P+P')/2;
    
    % State prediction
    %x = A*x + Bu + Bw*w_mean;
    %omgz embora não seja um output, esta a ser medido por isso em principio usa-se o valor medido do sensor e nao a estimativa do valor real 
    rollPonto=x(2) + x(7)*cos(x(1))*tan(x(3)) + x(4)*tan(x(3))*sin(x(1));
    pitchPonto=x(4)*cos( x(1)) - x(7)*sin( x(1));
    
    Ts = 0.0039;
    
    
   x(1)= rollPonto*Ts + x(1);
    x(3)= pitchPonto*Ts + x(3);
    
    x(2) = x(2);
    x(4) = x(4);
       
    x(5) = x(5);
    x(6) = x(6);
    
    x(7)=x(7);
    
end