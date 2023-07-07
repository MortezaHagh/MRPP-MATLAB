function model = createModel_mh_2(model)

    %% Algorithm
    % max_stall_count
    msc = 10;

    %% robots
    robot_count = 15;

    % dir: direction , r,l,u,d
    robo(1).dir = int32('r');
    robo(2).dir = int32('l');
    robo(3).dir = int32('l');
    robo(4).dir = int32('u');
    robo(5).dir = int32('u');
    robo(6).dir = int32('u');
    robo(7).dir = int32('r');
    robo(8).dir = int32('u');
    robo(9).dir = int32('d');
    robo(10).dir = int32('r');
    robo(11).dir = int32('u');
    robo(12).dir = int32('u');
    robo(13).dir = int32('u');
    robo(14).dir = int32('u');
    robo(15).dir = int32('u');
    robo(16).dir = int32('u');
    robo(17).dir = int32('u');
    robo(18).dir = int32('u');
    robo(19).dir = int32('d');
    robo(20).dir = int32('d');
    robo(21).dir = int32('d');
    robo(22).dir = int32('r');
    robo(23).dir = int32('u');
    robo(24).dir = int32('u');
    robo(25).dir = int32('u');
    robo(26).dir = int32('u');
    robo(27).dir = int32('u');
    robo(28).dir = int32('u');
    robo(29).dir = int32('u');

    % start & goal
    robo(1).xs = 1; robo(1).ys = 4;
    robo(1).xt = 21; robo(1).yt = 14;

    robo(2).xs = 25; robo(2).ys = 4;
    robo(2).xt = 4; robo(2).yt = 14;

    robo(3).xs = 5; robo(3).ys = 7;
    robo(3).xt = 1; robo(3).yt = 12;

    robo(4).xs = 20; robo(4).ys = 1;
    robo(4).xt = 5; robo(4).yt = 23;

    robo(5).xs = 5; robo(5).ys = 1;
    robo(5).xt = 21; robo(5).yt = 23;

    robo(6).xs = 12; robo(6).ys = 1;
    robo(6).xt = 12; robo(6).yt = 25;

    robo(7).xs = 1; robo(7).ys = 14;
    robo(7).xt = 12; robo(7).yt = 19;

    robo(8).xs = 2; robo(8).ys = 19;
    robo(8).xt = 24; robo(8).yt = 19;

    robo(9).xs = 4; robo(9).ys = 19;
    robo(9).xt = 21; robo(9).yt = 19;

    robo(10).xs = 1; robo(10).ys = 16;
    robo(10).xt = 25; robo(10).yt = 16;

    robo(11).xs = 8; robo(11).ys = 1;
    robo(11).xt = 8; robo(11).yt = 25;

    robo(12).xs = 17; robo(12).ys = 1;
    robo(12).xt = 17; robo(12).yt = 24;

    robo(13).xs = 1; robo(13).ys = 11;
    robo(13).xt = 24; robo(13).yt = 14;

    robo(14).xs = 15; robo(14).ys = 4;
    robo(14).xt = 6; robo(14).yt = 19;

    robo(15).xs = 2; robo(15).ys = 1;
    robo(15).xt = 25; robo(15).yt = 23;

    robo(16).xs = 24; robo(16).ys = 1;
    robo(16).xt = 1; robo(16).yt = 24;

    robo(17).xs = 6; robo(17).ys = 9;
    robo(17).xt = 17; robo(17).yt = 14;

    robo(18).xs = 21; robo(18).ys = 4;
    robo(18).xt = 6; robo(18).yt = 14;

    robo(19).xs = 25; robo(19).ys = 18;
    robo(19).xt = 10; robo(19).yt = 14;

    robo(20).xs = 10; robo(20).ys = 1;
    robo(20).xt = 10; robo(20).yt = 19;

    robo(21).xs = 4; robo(21).ys = 1;
    robo(21).xt = 4; robo(21).yt = 25;

    robo(22).xs = 10; robo(22).ys = 9;
    robo(22).xt = 24; robo(22).yt = 9;

    robo(23).xs = 17; robo(23).ys = 4;
    robo(23).xt = 15; robo(23).yt = 23;

    robo(24).xs = 21; robo(24).ys = 9;
    robo(24).xt = 17; robo(24).yt = 19;

    robo(25).xs = 13; robo(25).ys = 1;
    robo(25).xt = 13; robo(25).yt = 23;

    robo(26).xs = 17; robo(26).ys = 9;
    robo(26).xt = 15; robo(26).yt = 19;

    robo(27).xs = 19; robo(27).ys = 9;
    robo(27).xt = 11; robo(27).yt = 23;

    robo(28).xs = 22; robo(28).ys = 1;
    robo(28).xt = 6; robo(28).yt = 25;

    robo(29).xs = 1; robo(29).ys = 21;
    robo(29).xt = 23; robo(29).yt = 11;

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
    % close
    % plotModel(model);

end
