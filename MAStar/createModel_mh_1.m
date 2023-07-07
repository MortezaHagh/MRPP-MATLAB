function model = createModel_mh_1(model)

    %% Algorithm
    % max_stall_count
    msc = 10;

    %% robots
    robot_count = 3;

    % dir: direction
    robo(1).dir = -pi/2;
    robo(2).dir = -pi/2;
    robo(3).dir = -pi/2;

    % start & goal
    robo(1).xs = 1; robo(1).ys = 4;
    robo(1).xt = 21; robo(1).yt = 14;

    robo(2).xs = 25; robo(2).ys = 4;
    robo(2).xt = 4; robo(2).yt = 14;

    robo(3).xs = 5; robo(3).ys = 7;
    robo(3).xt = 1; robo(3).yt = 12;

    %% Area
    limArea = 28;
    xmin = -1; xmax = limArea;
    ymin = -1; ymax = limArea;

    % x_node_num=xmax;
    % y_node_num=ymax;

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

    for j = ymin:ymax

        for i = xmin:xmax
            adj{k, 1} = k; % node number
            adj{k, 2} = [i, j]; % node coordinates
            Nodes.cord(1:2, k) = [i, j]'; % node coordinates
            Nodes.number(1, k) = k; % node number
            Nodes.cost(1, k) = 0;

            for nr = 1:robot_count

                if i == robo(nr).xs && j == robo(nr).ys
                    robo(nr).startNode = k; % start node number
                elseif i == robo(nr).xt && j == robo(nr).yt
                    robo(nr).targetNode = k; % target (final) node number
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

    %% save model
    model.robot_count = robot_count;
    model.obstNode = obstNode;
    model.xc = xc; model.yc = yc;
    model.numOfObs = numel(xc);
    model.limArea = limArea;
    model.obst_r = obst_r;
    model.nodes = Nodes;
    model.xmin = xmin;
    model.xmax = xmax;
    model.ymin = ymin;
    model.ymax = ymax;
    model.adj = adj;
    model.robo = robo;
    model.msc = msc;

    %% plot model
    % plotModel(model);

end
