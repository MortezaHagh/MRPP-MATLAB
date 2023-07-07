function path = optimalPath_simple(Model, open, isPath, nr)

    path_nodes = Model.Robots(nr).startNode * [1, 1];
    parent_ind = 1;

    if isPath(nr) == 1
        % starting from the last (target) node
        path_nodes(1) = Model.Robots(nr).targetNode;
        i = 2;

        % Traverse Open and determine the parent nodes
        parent_ind = [open.List.nodeNumber] == path_nodes(1);
        parent_node = open.List(parent_ind).pNode;

        % going back to start node
        while parent_node ~= Model.Robots(nr).startNode
            path_nodes(i) = parent_node;
            parent_ind = [open.List.nodeNumber] == parent_node;
            parent_node = open.List(parent_ind).pNode;
            i = i + 1;
        end

    end

    path.nodeNumbers = [Model.Robots(nr).startNode, flip(path_nodes)];
    path.coords = nodes2coords(path.nodeNumbers, Model);
    path.dirs = nodes2dirs(path.nodeNumbers, Model);

end
