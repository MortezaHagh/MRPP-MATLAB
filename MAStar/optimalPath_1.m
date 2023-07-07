function path = optimalPath_1(Model, open, isPath, nr)

    if isPath(nr) == 1
        % starting from the last (target) node
        path_nodes(1) = Model.robo(nr).targetNode;
        i = 2;

        % Traverse Open and determine the parent nodes
        parent_ind = [open.list.nodeNumber] == path_nodes(1);
        parent_node = open.list(parent_ind).pnode;
        time = open.list(parent_ind).time;

        % going back to start node
        while parent_node ~= Model.robo(nr).startNode
            time = time - 1;
            path_nodes(i) = parent_node;
            parent_ind = [open.list.nodeNumber] == parent_node ...
                & [open.list.time] == time;
            v = 1:open.count;
            parent_ind = v(parent_ind);
            parent_ind = parent_ind(end);
            parent_node = open.list(parent_ind).pnode;
            i = i + 1;
        end

    else
        path_nodes = Model.robo(nr).startNode * [1, 1];
        parent_ind = 1;
    end

    % start stall count
    ssc = ones(1, open.list(parent_ind).time);

    if numel(ssc) > 1
        disp('')
    end

    path.nodeNumbers = [Model.robo(nr).startNode * ssc, flip(path_nodes)];
    path.coords = nodes2coords(path.nodeNumbers, Model);
    path.dirs = nodes2dirs(path.nodeNumbers, Model);

end
