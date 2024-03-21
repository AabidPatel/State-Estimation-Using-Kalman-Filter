function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 
    covar_aug = [covarPrev,zeros(15,12); zeros(12,15),eye(12,12)];
    sqrt_covar_aug = chol(covar_aug,'lower');
    
    x_a = [uPrev;zeros(12,1)];
    alpha = 0.001;
    k = 1;
    beta = 2;
    n = 27;
    lambda = (alpha)^2 *(n+k) - n;
    
    x_p = [];
    x_m = [];
    
    for i = 1:length(sqrt_covar_aug)
        X_aug_p = x_a + ((n + lambda)^(1/2))*(sqrt_covar_aug(:,i));
        X_aug_m = x_a - ((n + lambda)^(1/2))*(sqrt_covar_aug(:,i));
        x_p = [x_p,X_aug_p];
        x_m= [x_m, X_aug_m];
    end
    
    X_aug  = [x_a , x_p , x_m];
    x_dot_aug = [];
    xs = [];
    g = [0;0;-9.81];
    
    for j = 1:55
        n_g = X_aug(16:18,j); n_a = X_aug(19:21,j); nb_g = X_aug(22:24,j); nb_a = X_aug(25:27,j);
        roll  = X_aug(4,j); pitch = X_aug(5,j); yaw = X_aug(6,j); x3 = X_aug(7:9,j); x4 = X_aug(10:12,j); x5 = X_aug(13:15,j);
        wm_x = angVel(1,1); wm_y = angVel(2,1); wm_z = angVel(3,1); am_x = acc(1,1); am_y = acc(2,1); am_z = acc(3,1);
        wm = [wm_x; wm_y; wm_z]; am = [am_x; am_y; am_z];
    
        Gx2_inv =[(cos(yaw)*sin(pitch))/cos(pitch) (sin(pitch)*sin(yaw))/cos(pitch) 1 ; -sin(yaw) cos(yaw) 0; cos(yaw)/cos(pitch) sin(yaw)/cos(pitch) 0];
        Gx2_inv = flip(Gx2_inv);
    
        R_zyx = ([cos(yaw) -sin(yaw) 0 ; sin(yaw) cos(yaw) 0 ;0 0 1]*[cos(pitch) 0 sin(pitch) ; 0 1 0 ; -sin(pitch) 0 cos(pitch)]*[1 0 0 ; 0 cos(roll) -sin(roll) ; 0 sin(roll) cos(roll)]);
    
        x_dot = [x3 ; Gx2_inv*R_zyx*(wm - x4 - n_g) ; g+(R_zyx)*(am - x5 - n_a); nb_g ; nb_a];
        x_dot_aug =[x_dot_aug, x_dot];
    
        xs(1:9,j) = x_dot_aug(1:9,j)*dt + X_aug(1:9,j);
        xs(10:15,j)= x_dot_aug(10:15,j) + X_aug(10:15,j);
     
    end
    
    %Calculatin_g the Weights
    wo_c = (lambda/(n+lambda)) + (1-(alpha)^2 + beta);
    wi_c = 1/(2*(n + lambda));
    wo_m = lambda/(n + lambda);
    wi_m = 1/(2*(n + lambda));
    
    u0 = wo_m * xs(:,1);
    u = 0;
    
    for i =2:55
        u =  u + wi_m * xs(:,i);
    end
    
    uEst = u0 + u;
    
    c0 = wo_c*(xs(:,1) - uEst)*(xs(:,1) -uEst)';
    c = 0;
    
    for i = 2:55
        c =  c + wi_c*(xs(:,i) - uEst)*(xs(:,i)-uEst)';
    end
    
    covarEst = c0 + c;

end
 
