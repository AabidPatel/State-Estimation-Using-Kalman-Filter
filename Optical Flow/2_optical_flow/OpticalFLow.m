%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 4;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);


%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
k = [311.0520, 0, 201.8724;
    0, 311.3885, 113.6210; 
    0, 0, 1];

for n = 2:length(sampledData)
    %% Initalize Loop load images
    currImg = sampledData(n).img;
    prevImg = sampledData(n-1).img;
    dt = sampledData(n).t - sampledData(n-1).t;
    
    %% Detect good points
    prevCorners = detectFASTFeatures(prevImg);
    prevPoints = prevCorners.Location;
    prevPoints(:,3) = 1;
    prev_goodPoints = k\prevPoints';
    prev_goodPoints = prev_goodPoints';
    
    %% Initalize the tracker to the last frame.
    pointTracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 3);
    initialize(pointTracker, prevCorners.Location, prevImg);
    
    %% Find the location of the next points;
    [currPoints, validity] = step(pointTracker, currImg);
    % currPoints = currPoints(validity,:);
    currPoints(:,3) = 1;
    curr_goodPoints = k\currPoints';
    curr_goodPoints = curr_goodPoints';
    
    
    %% Calculate velocity
    [position, orientation, R_c2w] = estimatePose(sampledData, n);
    % Use a for loop
    A = [];
    B = [];
    p_dot = [];
    for i = 1: length(curr_goodPoints)
        x = curr_goodPoints(i,1);
        y = curr_goodPoints(i,2);
        optPos = curr_goodPoints;
        optVel = (curr_goodPoints - prev_goodPoints)/dt;
        optVel = optVel(:,1:2);
        optPos = optPos(:,1:2);
        x_prev = prev_goodPoints(i,1);
        y_prev = prev_goodPoints(i,2);
        p = [x;y;1];
        p_w = R_c2w*p;
        cos_theta = dot(p_w,[0,0,-1]);
        Z = position(3)/cos_theta;        

        A = [A;(1/Z)*[-1, 0, x; 0, -1, y]];
        B = [B;x*y, -(1+x^2), y; (1+y^2), -x*y, -x];
        p_dot = [p_dot;((x-x_prev)/dt); ((y-y_prev)/dt)];
    end
    
    %% Calculate Height
    H = [A,B];
    Vel_cwc = pinv(H) * p_dot;
    
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    % Velocity = velocityRANSAC(optVel,optPos,Z,R_c2w,e);
    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    s = [0, 0.03, 0; -0.03, 0, 0.04; 0 -0.04 0 ];
    Velocity = [R_c2w  -R_c2w*s; zeros(3,3) R_c2w]*Vel_cwc;
    
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
    estimatedV(:,n) = Velocity;
    if n>8
        estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)),3,7);
        estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)),3,7);
        estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)),3,7);
        estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)),3,7);
        estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)),3,7);
        estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)),3,7);
    end

    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    %estimatedV(:,n) = Velocity; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
