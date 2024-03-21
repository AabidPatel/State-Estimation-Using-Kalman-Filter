function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst,omg)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    %covar_aug = [covarPrev,zeros(15,12); zeros(12,15),eye(12,12)];

    sqrt_covar_aug = chol(covarEst,'lower');

    alpha = 0.001;
    k = 1;
    beta = 2;
    n = 15;
    lambda = (alpha)^2 *(n+k) - n;
    
    x_a = uEst;
    x_p = [];
    x_m = [];
    y_a = model(x_a,omg);
    y_p = [];
    y_m = [];
    
    for i = 1:length(sqrt_covar_aug)
        X_aug_p = x_a + ((n + lambda)^(1/2))*(sqrt_covar_aug(:,i));
        X_aug_m = x_a - ((n + lambda)^(1/2))*(sqrt_covar_aug(:,i));
        Y_p = model(X_aug_p,omg);
        Y_m = model(X_aug_m,omg);
        y_p = [y_p,Y_p];
        y_m = [y_m,Y_m];
        x_p = [x_p,X_aug_p];
        x_m= [x_m, X_aug_m];
    end
    
    X_aug  = [x_a , x_p , x_m];
    Y  = [y_a, y_p, y_m];

    wo_c = (lambda/(n+lambda)) + (1-(alpha)^2 + beta);
    wi_c = 1/(2*(n + lambda));
    wo_m = lambda/(n + lambda);
    wi_m = 1/(2*(n + lambda));

    R = eye(3,3) * 0.5;

    u = 0;
    u0 = wo_m * Y(:,1);
    for i = 2:31
        u = u + wi_m * Y(:,i);
    end
    u = u0 + u;

    covar_pred = 0;
    covar_pred_0 = wo_c * ((Y(:,1) - u) * (Y(:,1) - u)');
    for i = 2:31
        c = (Y(:,i) - u) * (Y(:,i) - u)';
        covar_pred = covar_pred + wi_c * c;
    end
    covar_pred = covar_pred_0 + covar_pred;
    covar_pred = covar_pred + R;

    cross_covar = 0;
    cross_covar_0 = wo_c * ((X_aug(:,1) - x_a) *(Y(:,1)-u)');
    for i = 2:31
        covar = (X_aug(:,i) - x_a) *(Y(:,i)-u)';
        cross_covar = cross_covar + wi_c * covar;
    end
    cross_covar = cross_covar_0 + cross_covar;

    Kt = cross_covar*inv(covar_pred);
    uCurr = uEst + Kt * (z_t' - u);
    covar_curr = covarEst - Kt * covar_pred * Kt';

    function [gu] = model(uEst,omg)
        R_cb = [0.7071, -0.7071, 0; -0.7071, -0.7071, 0; 0, 0, -1;];
        R_bw = [cos(uEst(5))*cos(uEst(6)), cos(uEst(6))*sin(uEst(5))*sin(uEst(4)) - cos(uEst(4))*sin(uEst(6)), sin(uEst(4))*sin(uEst(6)) + cos(uEst(4))*cos(uEst(6))*sin(uEst(5)); cos(uEst(5))*sin(uEst(6)), cos(uEst(4))*cos(uEst(6)) + sin(uEst(5))*sin(uEst(4))*sin(uEst(6)), cos(uEst(4))*sin(uEst(5))*sin(uEst(6)) - cos(uEst(6))*sin(uEst(4)); -sin(uEst(5)), cos(uEst(5))*sin(uEst(4)), cos(uEst(5))*cos(uEst(4))]';
        s = [0, 0.03, 0.0283; -0.03, 0, -0.0283; -0.0283, 0.0283, 0;];
        gu = R_cb * R_bw * uEst(7:9,1) - R_cb * s *R_cb' * omg'  + zeros(3,1);
    end
end

