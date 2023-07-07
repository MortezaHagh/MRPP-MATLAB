function sorted_list = priority(robots, robot_list, Model, nodes)

    h = zeros(numel(robot_list), 1);
    j = 1;

    for i = robot_list
        xyp = Model.nodes.cord(:, nodes(i));
        xyt = Model.nodes.cord(:, robots(i).targetNode);
        h(j) = calDistance(xyp(1), xyp(2), xyt(1), xyt(2), Model.dist_type);
        j = j + 1;
    end

    [~, ind] = sort(h);
    sorted_list = robot_list(ind);
end
