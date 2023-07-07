function path = optimalPath_1(Model, open, isPath, nr)

    if isPath(nr) == 1
        % starting from the last (target) node
        path_nodes(1) = Model.Robots(nr).targetNode;
        i = 2;

        % Traverse Open and determine the parent nodes
        parent_ind = [open.List.nodeNumber] == path_nodes(1);
        parent_node = open.List(parent_ind).pNode;
        time = open.List(parent_ind).time;

        % going back to start node
        while parent_node ~= Model.Robots(nr).startNode
            time = time - 1;
            path_nodes(i) = parent_node;
            parent_ind = [open.List.nodeNumber] == parent_node ...
                & [open.List.time] == time;
            v = 1:open.count;
            parent_ind = v(parent_ind);
            parent_ind = parent_ind(end);
            parent_node = open.List(parent_ind).pNode;
            i = i + 1;
        end

    else
        path_nodes = Model.Robots(nr).startNode * [1, 1];
        parent_ind = 1;
    end

    % start stall count
    ssc = ones(1, open.List(parent_ind).time);

    if numel(ssc) > 1
        disp('')
    end

    path.nodeNumbers = [Model.Robots(nr).startNode * ssc, flip(path_nodes)];
    path.coords = nodes2coords(path.nodeNumbers, Model);
    path.dirs = nodes2dirs(path.nodeNumbers, Model);

end
