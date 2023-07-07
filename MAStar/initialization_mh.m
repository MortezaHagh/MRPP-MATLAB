function [closed, open, topnodes, Robots, paths, closed_init] = initialization_mh(Model)

    % Initialization and Parameters

    % nodes & obstacles
    obstNode = Model.obstNode;
    num_Of_Obs = Model.numOfObs;

    % robots
    Robots = Model.Robots;
    robot_count = Model.robot_count;

    % closed structure
    cemp.count = 0;
    cemp.nodeNumbers = [];
    closed = repmat(cemp, robot_count, 1);

    % open structure
    oemp.count = 0;
    oemp.list = [];
    open = repmat(oemp, robot_count, 1);

    % optimal path structure
    p.coords = [];
    p.nodeNumbers = [];
    p.dirs = [];
    paths = repmat(p, robot_count, 1);

    %%% initialization
    for nr = 1:robot_count

        % closed: put all obstacles on the Closed list
        closed(nr).count = closed(nr).count + num_Of_Obs;
        closed(nr).nodeNumbers = [closed(nr).nodeNumbers, obstNode];

        % add target nodes to closed lists
        %     rv=1:robot_count;
        %     for nnr = rv(rv~=nr)
        %         closed(nnr).count = closed(nnr).count+1;
        %         closed(nnr).nodeNumbers(end+1) = Robots(nr).targetNode;
        %     end

        % set the starting node (topnode) as the first node in Open
        topnode.visited = 1;
        topnode.nodeNumber = Robots(nr).startNode;
        topnode.pnode = Robots(nr).startNode;
        topnode.dir = Robots(nr).dir;
        topnode.cost_g = 0;
        cost_h = calDistance(Robots(nr).xs, Robots(nr).ys, Robots(nr).xt, Robots(nr).yt, Model.distType) * 2;
        topnode.cost_f = topnode.cost_g + cost_h;
        topnode.time = 0;
        topnode.tag = 1;

        % insert start node in open list
        open(nr).count = 1;
        open(nr).list = topnode;

        % add last node (hear start node) to Closed
        closed(nr).count = closed(nr).count + 1;
        closed(nr).nodeNumbers(end + 1) = topnode.nodeNumber;
    end

    % topnodes
    topnodes = [open.list];

    closed_init = closed;
end
