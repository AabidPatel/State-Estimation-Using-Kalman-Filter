function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    %R_c2w = Rotation which defines camera to world frame
    res = getCorner(data(t).id);
    A = [];

    for i = 1:length(data(t).id)
        x0 = (res.P0(1,i));
        y0 = (res.P0(2,i));
        x1 = (res.P1(1,i));
        y1 = (res.P1(2,i));
        x2 = (res.P2(1,i));
        y2 = (res.P2(2,i));
        x3 = (res.P3(1,i));
        y3 = (res.P3(2,i));
        x4 = (res.P4(1,i));
        y4 = (res.P4(2,i));

        x_0 = data(t).p0(1,i);
        y_0 = data(t).p0(2,i);
        x_1 = data(t).p1(1,i);
        y_1 = data(t).p1(2,i);
        x_2 = data(t).p2(1,i);
        y_2 = data(t).p2(2,i);
        x_3 = data(t).p3(1,i);
        y_3 = data(t).p3(2,i);
        x_4 = data(t).p4(1,i);
        y_4 = data(t).p4(2,i);

        A1 = [x0, y0, 1, 0, 0, 0, (-x_0*x0), (-x_0*y0), -x_0;
              0, 0, 0, x0, y0, 1, (-y_0*x0), (-y_0*y0), -y_0];

        A2 = [x1, y1, 1, 0, 0, 0, (-x_1*x1), (-x_1*y1), -x_1;
              0, 0, 0, x1, y1, 1, (-y_1*x1), (-y_1*y1), -y_1];

        A3 = [x2, y2, 1, 0, 0, 0, (-x_2*x2), (-x_2*y2), -x_2;
              0, 0, 0, x2, y2, 1, (-y_2*x2), (-y_2*y2), -y_2];

        A4 = [x3, y3, 1, 0, 0, 0, (-x_3*x3), (-x_3*y3), -x_3;
              0, 0, 0, x3, y3, 1, (-y_3*x3), (-y_3*y3), -y_3];

        A5 = [x4, y4, 1, 0, 0, 0, (-x_4*x4), (-x_4*y4), -x_4;
              0, 0, 0, x4, y4, 1, (-y_4*x4), (-y_4*y4), -y_4];

        A0 = [A1; A2; A3; A4; A5];

        A = [A; A0];
    end

    [U, S, V] = svd(A);

    h = V(:,9)*sign(V(9,9));

    h = reshape(h,3,3)';

    K = [311.0520, 0, 201.8724;
        0, 311.3885, 113.6210;
        0, 0, 1];

    h = K\h;

    h1 = h(:,1);
    h2 = h(:,2);
    h3 = h(:,3);

    R1_cap = h1/norm(h1);
    R2_cap = h2/norm(h2);
    R3_cap = cross(R1_cap, R2_cap);
    T = h3/norm(h1);

    R = [R1_cap, R2_cap, R3_cap];

    [u, s, v] = svd(R);
    x = [1,0,0;
        0,1,0;
        0,0,det(u*v')];

    R = u*x*v';
    H = [R,T;0,0,0,1];
    Rwc = rotz(-45)*rotx(180);
    Twc = [-0.04; 0.0; -0.03];
    Hwc = [Rwc, Twc; 0,0,0,1];
    Hcww = inv(H);
    Hw = H\Hwc;
    R_c2w = Hcww(1:3,1:3);
    orientation = rotm2eul(Hw(1:3,1:3),'ZYX');
    position = Hw(1:3,4);
end