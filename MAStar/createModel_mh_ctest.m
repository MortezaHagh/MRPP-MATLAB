function model = createModel_mh_ctest(model)

    %% Algorithm
    % max_stall_count
    msc = 10;

    %% robots
    robot_count = 3;

    % dir: direction , r,l,u,d
    robo(1).dir = int32('d');
    robo(2).dir = int32('d');
    robo(3).dir = int32('d');

    % start & goal
    robo(1).xs = 1; robo(1).ys = 0;
    robo(1).xt = 0; robo(1).yt = 3;

    robo(2).xs = -1; robo(2).ys = 0;
    robo(2).xt = 1; robo(2).yt = 3;

    robo(3).xs = 0; robo(3).ys = -1;
    robo(3).xt = 1; robo(3).yt = 4;

    %% Area
    limArea = 4;
    xmin = -1; xmax = limArea;
    ymin = -1; ymax = limArea;

    % x_node_num=xmax;
    % y_node_num=ymax;

    %%% Obstacle
    obst_r = 0.25;

    xc = [1 -1 1 2 2 2 -1];
    yc = [-1 1 1 1 0 -1 -1];

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
    % close
    % plotModel(model);

end
