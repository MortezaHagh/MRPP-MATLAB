function Model = createModel_ctest(Model)

    %% Algorithm
    % max_stall_count
    msc = 10;

    %% Robots
    robotCount = 3;

    % dir: direction
    Robots(1).dir = -pi / 2;
    Robots(2).dir = -pi / 2;
    Robots(3).dir = -pi / 2;

    % start & goal
    Robots(1).xs = 1; Robots(1).ys = 0;
    Robots(1).xt = 0; Robots(1).yt = 3;

    Robots(2).xs = -1; Robots(2).ys = 0;
    Robots(2).xt = 1; Robots(2).yt = 3;

    Robots(3).xs = 0; Robots(3).ys = -1;
    Robots(3).xt = 1; Robots(3).yt = 4;

    %% Area
    limArea = 4;
    xMin = -1; xMax = limArea;
    yMin = -1; yMax = limArea;

    % x_node_num=xMax;
    % y_node_num=yMax;

    %%% Obstacle
    obst_r = 0.25;

    xc = [1 -1 1 2 2 2 -1];
    yc = [-1 1 1 1 0 -1 -1];

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

            for nr = 1:robotCount

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

    Model.robotCount = robotCount;

    Model.Map = Map;
    Model.Obsts = Obsts;

    %% save Model
    Model.robotCount = robotCount;
    Model.obstNode = obstNode;
    Model.xc = xc; Model.yc = yc;
    Model.obstCount = numel(xc);
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
