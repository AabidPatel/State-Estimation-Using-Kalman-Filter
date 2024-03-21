function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    I = zeros(6,15);
    I(1:6,1:6)= eye(6,6);

    z = I * uEst + zeros(6,1);
    Ct = I ;
    Wt = eye(6,6);                                                                      
    R = eye(6,6) * 0.1;
    Kt = covarEst * Ct' /(Ct * covarEst* Ct' + Wt * R * Wt');

    uCurr = uEst + Kt * (z_t - z);
    covar_curr = covarEst- Kt* Ct * covarEst;
    
end

