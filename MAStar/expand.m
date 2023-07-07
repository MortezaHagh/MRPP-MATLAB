function expand = expand(topNode, Closed, Model, nr)
    % expand a node and find feasible successors
    % nodeNumber pNode gCost fCost dir

    robot = Model.Robots(nr);

    % neghbors from model
    nNeighbors = Model.Neighbors(topNode.nodeNumber).count;
    Neighbors = Model.Neighbors(topNode.nodeNumber).List;

    nExpand = 0;

    for iNeighbor = 1:nNeighbors
        % neighbor
        newX = Neighbors(iNeighbor).x;
        newY = Neighbors(iNeighbor).y;
        newDir = Neighbors(iNeighbor).dir;
        newCost = Neighbors(iNeighbor).cost;
        newNodeNumber = Neighbors(iNeighbor).nodeNumber;

        % add neighbor (newNode) to list if it is not in Closed list
        if ~any(newNodeNumber == Closed.nodeNumbers)
            nExpand = nExpand + 1;
            list(nExpand).visited = 0;
            list(nExpand).nodeNumber = newNodeNumber;
            list(nExpand).pNode = topNode.nodeNumber;
            list(nExpand).gCost = topNode.gCost + newCost;
            hCost = calDistance(robot.xt, robot.yt, newX, newY, Model.distType);
            list(nExpand).fCost = list(nExpand).gCost + 1 * hCost;
            list(nExpand).dir = newDir;
            list(nExpand).time = topNode.time + 1;
            list(nExpand).tag = 1;
        end

    end

    % Neighbors
    expand.count = nExpand;

    if nExpand ~= 0
        %         dTheta = angdiff(topNode.dir*ones(1, nExpand), [list.dir]);
        %         dTheta = abs(dTheta);
        %         [~, sortInds] = sort(dTheta);
        %         list = list(sortInds);
        expand.List = list;
    else
        expand.List = [];
    end

end
