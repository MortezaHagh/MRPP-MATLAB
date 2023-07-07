function Model = createModel_mh_2(Model)

    %% Algorithm
    % max_stall_count
    msc = 10;

    %% robots
    robot_count = 10;

    % dir: direction
    Robots(1).dir = 0;
    Robots(2).dir = -pi;
    Robots(3).dir = -pi;
    Robots(4).dir = pi / 2;
    Robots(5).dir = pi / 2;
    Robots(6).dir = pi / 2;
    Robots(7).dir = 0;
    Robots(8).dir = pi / 2;
    Robots(9).dir = -pi / 2;
    Robots(10).dir = 0;
    Robots(11).dir = pi / 2;
    Robots(12).dir = pi / 2;
    Robots(13).dir = pi / 2;
    Robots(14).dir = pi / 2;
    Robots(15).dir = pi / 2;
    Robots(16).dir = pi / 2;
    Robots(17).dir = pi / 2;
    Robots(18).dir = pi / 2;
    Robots(19).dir = -pi / 2;
    Robots(20).dir = -pi / 2;
    Robots(21).dir = -pi / 2;
    Robots(22).dir = 0;
    Robots(23).dir = pi / 2;
    Robots(24).dir = pi / 2;
    Robots(25).dir = pi / 2;
    Robots(26).dir = pi / 2;
    Robots(27).dir = pi / 2;
    Robots(28).dir = pi / 2;
    Robots(29).dir = pi / 2;

    % start & goal
    Robots(1).xs = 1; Robots(1).ys = 4;
    Robots(1).xt = 21; Robots(1).yt = 14;

    Robots(2).xs = 25; Robots(2).ys = 4;
    Robots(2).xt = 4; Robots(2).yt = 14;

    Robots(3).xs = 5; Robots(3).ys = 7;
    Robots(3).xt = 1; Robots(3).yt = 12;

    Robots(4).xs = 20; Robots(4).ys = 1;
    Robots(4).xt = 5; Robots(4).yt = 23;

    Robots(5).xs = 5; Robots(5).ys = 1;
    Robots(5).xt = 21; Robots(5).yt = 23;

    Robots(6).xs = 12; Robots(6).ys = 1;
    Robots(6).xt = 12; Robots(6).yt = 25;

    Robots(7).xs = 1; Robots(7).ys = 14;
    Robots(7).xt = 12; Robots(7).yt = 19;

    Robots(8).xs = 2; Robots(8).ys = 19;
    Robots(8).xt = 24; Robots(8).yt = 19;

    Robots(9).xs = 4; Robots(9).ys = 19;
    Robots(9).xt = 21; Robots(9).yt = 19;

    Robots(10).xs = 1; Robots(10).ys = 16;
    Robots(10).xt = 25; Robots(10).yt = 16;

    Robots(11).xs = 8; Robots(11).ys = 1;
    Robots(11).xt = 8; Robots(11).yt = 25;

    Robots(12).xs = 17; Robots(12).ys = 1;
    Robots(12).xt = 17; Robots(12).yt = 24;

    Robots(13).xs = 1; Robots(13).ys = 11;
    Robots(13).xt = 24; Robots(13).yt = 14;

    Robots(14).xs = 15; Robots(14).ys = 4;
    Robots(14).xt = 6; Robots(14).yt = 19;

    Robots(15).xs = 2; Robots(15).ys = 1;
    Robots(15).xt = 25; Robots(15).yt = 23;

    Robots(16).xs = 24; Robots(16).ys = 1;
    Robots(16).xt = 1; Robots(16).yt = 24;

    Robots(17).xs = 6; Robots(17).ys = 9;
    Robots(17).xt = 17; Robots(17).yt = 14;

    Robots(18).xs = 21; Robots(18).ys = 4;
    Robots(18).xt = 6; Robots(18).yt = 14;

    Robots(19).xs = 25; Robots(19).ys = 18;
    Robots(19).xt = 10; Robots(19).yt = 14;

    Robots(20).xs = 10; Robots(20).ys = 1;
    Robots(20).xt = 10; Robots(20).yt = 19;

    Robots(21).xs = 4; Robots(21).ys = 1;
    Robots(21).xt = 4; Robots(21).yt = 25;

    Robots(22).xs = 10; Robots(22).ys = 9;
    Robots(22).xt = 24; Robots(22).yt = 9;

    Robots(23).xs = 17; Robots(23).ys = 4;
    Robots(23).xt = 15; Robots(23).yt = 23;

    Robots(24).xs = 21; Robots(24).ys = 9;
    Robots(24).xt = 17; Robots(24).yt = 19;

    Robots(25).xs = 13; Robots(25).ys = 1;
    Robots(25).xt = 13; Robots(25).yt = 23;

    Robots(26).xs = 17; Robots(26).ys = 9;
    Robots(26).xt = 15; Robots(26).yt = 19;

    Robots(27).xs = 19; Robots(27).ys = 9;
    Robots(27).xt = 11; Robots(27).yt = 23;

    Robots(28).xs = 22; Robots(28).ys = 1;
    Robots(28).xt = 6; Robots(28).yt = 25;

    Robots(29).xs = 1; Robots(29).ys = 21;
    Robots(29).xt = 23; Robots(29).yt = 11;

    %% Area
    limArea = 28;
    xMin = -1; xMax = limArea;
    yMin = -1; yMax = limArea;

    % x_node_num=xMax;
    % y_node_num=yMax;

    %%% Obstacle
    obst_r = 0.25;
    xc1 = [4 4 4 6 6 6 8 8 8 10 10 10 12 12 12];
    yc1 = [4 5 6 4 5 6 4 5 6 4 5 6 4 5 6];

    xc2 = [xc1 xc1 xc1 xc1];
    yc2 = [yc1 yc1 + 6 yc1 + 12 yc1 + 18];

    xc3 = xc2 + 12;
    yc3 = yc2;

    xc = [xc2 xc3];
    yc = [yc2 yc3];

    %% Nodes & Adj
    k = 1;
    adj = cell(1, 1);

    for j = yMin:yMax

        for i = xMin:xMax
            adj{k, 1} = k; % node number
            adj{k, 2} = [i, j]; % node coordinates
            Nodes.cord(1:2, k) = [i, j]'; % node coordinates
            Nodes.number(1, k) = k; % node number
            Nodes.cost(1, k) = 0;

            for nr = 1:robot_count

                if i == Robots(nr).xs && j == Robots(nr).ys
                    Robots(nr).startNode = k; % start node number
                elseif i == Robots(nr).xt && j == Robots(nr).yt
                    Robots(nr).targetNode = k; % target (final) node number
                end

            end

            k = k + 1;
        end

    end

    Nodes.count = k - 1;

    % obstacle node numbers
    obstNode = zeros(1, length(xc));

    for i = 1:length(xc)

        for j = 1:size(Nodes.number, 2)

            if xc(i) == Nodes.cord(1, j) && yc(i) == Nodes.cord(2, j)
                obstNode(i) = Nodes.number(j);
            end

        end

    end

    %% Map Obsts
    Map.lim = limArea;
    Map.xMin = xMin;
    Map.xMax = xMax;
    Map.yMin = yMin;
    Map.yMax = yMax;
    Map.nX = Map.xMax - Map.xMin + 1;
    Map.nY = Map.yMax - Map.yMin + 1;

    Obsts.count = numel(obstNode);
    Obsts.x = xc;
    Obsts.y = yc;
    Obsts.nodeNumber = obstNode;
    Obsts.r = obst_r;

    Model.robotCount = robot_count;

    Model.Map = Map;
    Model.Obsts = Obsts;

    %% save Model
    Model.robot_count = robot_count;
    Model.obstNode = obstNode;
    Model.xc = xc; Model.yc = yc;
    Model.numOfObs = numel(xc);
    Model.limArea = limArea;
    Model.obst_r = obst_r;
    Model.Nodes = Nodes;
    Model.xMin = xMin;
    Model.xMax = xMax;
    Model.yMin = yMin;
    Model.yMax = yMax;
    Model.adj = adj;
    Model.Robots = Robots;
    Model.msc = msc;

end
