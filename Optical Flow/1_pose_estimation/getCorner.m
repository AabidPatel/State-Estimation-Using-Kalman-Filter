function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
    row = 12;
    column = 9;
    x = 0;
    y = 0;
    counter = 0;
    P0 = [];
    P1 = [];
    P2 = [];
    P3 = [];
    P4 = [];
    id = id + 1;

    for i = 1:column
        for j = 1:row
            p4 = [x; y];
            p1 = [x+0.152; y];
            p2 = [x+0.152; y+0.152];
            p3 = [x; y+0.152];
            p0 = [p2(1)-0.152/2; p2(2)-0.152/2];
            x = x + 2*(0.152);
            P0 = [P0, p0];
            P1 = [P1, p1];
            P2 = [P2, p2];
            P3 = [P3, p3];
            P4 = [P4, p4];
            counter = counter + 1;
        end
        % Below we account for the change in the dimension after 3 columns
        if counter >= 0 && counter < 36
            x = 0;
            y = y + 2*(0.152);
        elseif counter == 36
            x = 0;
            y = 0.938;
        elseif counter == 48
            x = 0;
            y = 1.242;
        elseif counter == 60
            x = 0;
            y = 1.546;
        elseif counter == 72
            x = 0;
            y = 1.876;
        elseif counter == 84
            x = 0;
            y = 2.18;
        elseif counter == 96
            x = 0;
            y = 2.484;
        end
    end

    % res = [P0(:,id); P1(:,id); P2(:,id); P3(:,id); P4(:,id)];
    res = struct('P0', P0(:,id), 'P1', P1(:,id), 'P2', P2(:,id), 'P3', P3(:,id), 'P4', P4(:,id));
    
end