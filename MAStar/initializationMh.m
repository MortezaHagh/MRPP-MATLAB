function [closed, open, topnodes, Robots, paths, closed_init] = initializationMh(Model)

    % Initialization and Parameters

    % nodes & obstacles
    obstNode = Model.obstNode;
    num_Of_Obs = Model.numOfObs;

    % robots
    Robots = Model.Robots;
    robotCount = Model.robotCount;

    % closed structure
    cemp.count = 0;
    cemp.nodeNumbers = [];
    closed = repmat(cemp, robotCount, 1);

    % open structure
    oemp.count = 0;
    oemp.List = [];
    open = repmat(oemp, robotCount, 1);

    % optimal path structure
    p.coords = [];
    p.nodeNumbers = [];
    p.dirs = [];
    paths = repmat(p, robotCount, 1);

    %%% initialization
    for nr = 1:robotCount

        % closed: put all obstacles on the Closed list
        closed(nr).count = closed(nr).count + num_Of_Obs;
        closed(nr).nodeNumbers = [closed(nr).nodeNumbers, obstNode];

        % add target nodes to closed lists
        %     rv=1:robotCount;
        %     for nnr = rv(rv~=nr)
        %         closed(nnr).count = closed(nnr).count+1;
        %         closed(nnr).nodeNumbers(end+1) = Robots(nr).targetNode;
        %     end

        % set the starting node (topNode) as the first node in Open
        topNode.visited = 1;
        topNode.nodeNumber = Robots(nr).startNode;
        topNode.pNode = Robots(nr).startNode;
        topNode.dir = Robots(nr).dir;
        topNode.gCost = 0;
        hCost = calDistance(Robots(nr).xs, Robots(nr).ys, Robots(nr).xt, Robots(nr).yt, Model.distType) * 2;
        topNode.fCost = topNode.gCost + hCost;
        topNode.time = 0;
        topNode.tag = 1;

        % insert start node in open list
        open(nr).count = 1;
        open(nr).List = topNode;

        % add last node (hear start node) to Closed
        closed(nr).count = closed(nr).count + 1;
        closed(nr).nodeNumbers(end + 1) = topNode.nodeNumber;
    end

    % topnodes
    topnodes = [open.List];

    closed_init = closed;
end
