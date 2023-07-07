function neighbors = neighbors8(topnode, closed, Model, nr)
    % node pnode cost_g cost_f dir

    xy = Model.Nodes.cord(:, topnode.nodeNumber);
    robot = Model.robo(nr);
    x = xy(1);
    y = xy(2);
    nc = 0; % neighbors.count

    dxdy = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
    dirs = [0, pi / 4, pi / 2, 3 * pi / 4, pi, 5 * pi / 4, 3 * pi / 2, 7 * pi / 4];

    for k = 1:8
        i = dxdy(k, 1);
        j = dxdy(k, 2);

        % dir
        nn_dir = dirs(k);

        % nn: new node
        if (i ~= j || i ~= 0) % &&
            % The node itself is not its successor
            %(i==0 || j==0)  % eliminate corner nodes -> 4 node
            nn_x = x +i;
            nn_y = y +j;

            % check if the new node is within limits
            if ((nn_x >= Model.xMin && nn_x <= Model.xMax) && (nn_y >= Model.yMin && nn_y <= Model.yMax))
                new_node = topnode.nodeNumber + i + (j * (Model.xMax - Model.xMin + 1));

                % check if it is in Closed list
                if ~any(new_node == closed.nodeNumbers)
                    nc = nc + 1;
                    list(nc).visited = 0;
                    list(nc).nodeNumber = new_node;
                    list(nc).pnode = topnode.nodeNumber;
                    list(nc).cost_g = topnode.cost_g + calDistance(x, y, nn_x, nn_y, Model.dist_type);
                    cost_h = calDistance(robot.xt, robot.yt, nn_x, nn_y, Model.dist_type) * 2;
                    list(nc).cost_f = list(nc).cost_g + cost_h;
                    list(nc).dir = nn_dir;
                    list(nc).time = topnode.time + 1;
                    list(nc).tag = 1;
                end

            end

        end

    end

    neighbors.count = nc;

    if nc ~= 0
        neighbors.list = list;
    else
        neighbors.list = [];
    end

end
