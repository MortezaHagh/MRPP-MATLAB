function neighbors = neighbors4(topnode, closed, model, nr)
    % arrange neighbors based on robot direction
    % node pnode cost_g cost_f dir

    xy = model.nodes.cord(:, topnode.node);
    robot = model.robo(nr);
    dir = topnode.dir;
    x = xy(1);
    y = xy(2);
    nc = 0; % neighbors.count

    % prioritize expanded node based on direction
    q1 = [1 0; 0 1; 0 -1; -1 0];
    q2 = [-1 0; 0 1; 0 -1; 1 0];
    q3 = [0 1; 1 0; -1 0; 0 -1];
    q4 = [0 -1; 1 0; -1 0; 0 1];
    if dir == 0 || dir == 2 * pi; qs = q1; end % 0 1
    if dir == pi || dir == -pi; qs = q2; end % -pi 3
    if dir == pi / 2; qs = q3; end % pi/2 2
    if dir == -pi / 2 || dir == 3 * pi / 2; qs = q4; end % -pi/2 4

    for k = 1:4
        % nn: new node
        i = qs(k, 1); j = qs(k, 2);

        % direction
        if all([i, j] == [1, 0]); nn_dir = 0; end
        if all([i, j] == [-1, 0]); nn_dir = pi; end
        if all([i, j] == [0, 1]); nn_dir = pi / 2; end
        if all([i, j] == [0, -1]); nn_dir = 3 * pi / 2; end

        nn_x = x + i;
        nn_y = y + j;

        % check if the new node is within limits
        if ((nn_x >= model.xmin && nn_x <= model.xmax) && (nn_y >= model.ymin && nn_y <= model.ymax))
            new_node = topnode.node + i + (j * (model.xmax - model.xmin + 1));

            % check if it is in Closed list
            if ~any(new_node == closed.nodes)
                nc = nc + 1;
                list(nc).visited = 0;
                list(nc).node = new_node;
                list(nc).pnode = topnode.node;
                list(nc).cost_g = topnode.cost_g + calDistance(x, y, nn_x, nn_y, model.dist_type);
                cost_h = calDistance(robot.xt, robot.yt, nn_x, nn_y, model.dist_type);
                list(nc).cost_f = list(nc).cost_g + cost_h;
                list(nc).dir = nn_dir;
                list(nc).time = topnode.time + 1;
                list(nc).tag = 1;
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
