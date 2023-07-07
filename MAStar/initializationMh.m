function [Closed, Open, topnodes, Robots, Paths, closedInit] = initializationMh(Model)

    % Initialization and Parameters

    % nodes & obstacles
    obstNode = Model.obstNode;
    obstCount = Model.obstCount;

    % Robots
    Robots = Model.Robots;
    robotCount = Model.robotCount;

    % Closed structure
    cemp.count = 0;
    cemp.nodeNumbers = [];
    Closed = repmat(cemp, robotCount, 1);

    % Open structure
    oemp.count = 0;
    oemp.List = [];
    Open = repmat(oemp, robotCount, 1);

    % optimal path structure
    p.coords = [];
    p.nodeNumbers = [];
    p.dirs = [];
    Paths = repmat(p, robotCount, 1);

    %%% initialization
    for nr = 1:robotCount

        % Closed: put all obstacles on the Closed list
        Closed(nr).count = Closed(nr).count + obstCount;
        Closed(nr).nodeNumbers = [Closed(nr).nodeNumbers, obstNode];

        % add target nodes to Closed lists
        %     rv=1:robotCount;
        %     for nnr = rv(rv~=nr)
        %         Closed(nnr).count = Closed(nnr).count+1;
        %         Closed(nnr).nodeNumbers(end+1) = Robots(nr).targetNode;
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

        % insert start node in Open list
        Open(nr).count = 1;
        Open(nr).List = topNode;

        % add last node (hear start node) to Closed
        Closed(nr).count = Closed(nr).count + 1;
        Closed(nr).nodeNumbers(end + 1) = topNode.nodeNumber;
    end

    % topnodes
    topnodes = [Open.List];

    closedInit = Closed;
end
