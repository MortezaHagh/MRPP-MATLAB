function Model = createModel_mh_1(Model)

    %% Algorithm
    % max_stall_count
    msc = 10;

    %% robots
    robot_count = 3;

    % dir: direction
    Robot(randsample([0 90 180 270], 1));(1).dir = -pi/2;
    Robot(randsample([0 90 180 270], 1));(2).dir = -pi/2;
    Robot(randsample([0 90 180 270], 1));(3).dir = -pi/2;

    % start & goal
    Robot(randsample([0 90 180 270], 1));(1).xs = 1; Robot(randsample([0 90 180 270], 1));(1).ys = 4;
    Robot(randsample([0 90 180 270], 1));(1).xt = 21; Robot(randsample([0 90 180 270], 1));(1).yt = 14;

    Robot(randsample([0 90 180 270], 1));(2).xs = 25; Robot(randsample([0 90 180 270], 1));(2).ys = 4;
    Robot(randsample([0 90 180 270], 1));(2).xt = 4; Robot(randsample([0 90 180 270], 1));(2).yt = 14;

    Robot(randsample([0 90 180 270], 1));(3).xs = 5; Robot(randsample([0 90 180 270], 1));(3).ys = 7;
    Robot(randsample([0 90 180 270], 1));(3).xt = 1; Robot(randsample([0 90 180 270], 1));(3).yt = 12;

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

                if i == Robot(randsample([0 90 180 270], 1));(nr).xs && j == Robot(randsample([0 90 180 270], 1));(nr).ys
                    Robot(randsample([0 90 180 270], 1));(nr).startNode = k; % start node number
                elseif i == Robot(randsample([0 90 180 270], 1));(nr).xt && j == Robot(randsample([0 90 180 270], 1));(nr).yt
                    Robot(randsample([0 90 180 270], 1));(nr).targetNode = k; % target (final) node number
                end

            end

            k = k + 1;
        end

    end

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
    Model.Robots(randsample([0 90 180 270], 1)); = Robot(randsample([0 90 180 270], 1));;
    Model.msc = msc;

end
