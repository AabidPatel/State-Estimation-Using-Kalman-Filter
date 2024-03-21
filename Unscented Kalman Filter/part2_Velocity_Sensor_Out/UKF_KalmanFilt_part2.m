clear; % Clear variables
addpath('../data')
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.01*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0;
vel = proj2Data.linearVel;
angVel2 = proj2Data.angVel;
%% Calculate Kalmann Filter
for i = 1:length(sampledTime)
    %% FILL IN THE FOR LOOP
    acc = sampledData(i).acc;
    angVel= sampledData(i).omg;
    currTime = sampledTime(i);
    dt  = currTime - prevTime;
    z_t = vel(i,:);
    [covarEst,uEst] =  pred_step(uPrev,covarPrev,angVel,acc,dt);
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst,angVel2(i,:));
    % updating Vicon Initial state and Covariance constant
    uPrev = uCurr;
    covarPrev = covar_curr;
    % updating the last time step
    prevTime = currTime;
    savedStates(:,i) = uCurr;
end

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);