function Model = createModelFromMap(MapInput, Model)
    % Create Model from Map

    %% Map
    [H, W] = size(MapInput);
    Map.xMin = 0;
    Map.yMin = 0;
    Map.xMax = W - 1;
    Map.yMax = H - 1;
    Map.lim = max(W, H);
    Map.nX = Map.xMax - Map.xMin + 1;
    Map.nY = Map.yMax - Map.yMin + 1;

    %% Obstacles

    Obsts.r = 0.25;
    Obsts.x = [];
    Obsts.y = [];
    Obsts.nodeNumber = [];
    Nodes.count = H * W;
    Nodes.cord = zeros(2, Nodes.count);
    Nodes.number = zeros(1, Nodes.count);

    iNodeNumber = 1;

    for i = 1:H

        for j = 1:W
            Nodes.number(1, iNodeNumber) = iNodeNumber;
            Nodes.cord(:, iNodeNumber) = [j - 1, i - 1]';

            if MapInput(i, j) == 0
                Obsts.x = [Obsts.x j - 1];
                Obsts.y = [Obsts.y i - 1];
                Obsts.nodeNumber = [Obsts.nodeNumber iNodeNumber];
            end

            iNodeNumber = iNodeNumber + 1;
        end

    end

    Obsts.count = numel(Obsts.nodeNumber);

    %% edge costs, G, RHS, GV

    switch Model.adjType
        case '4adj'
            ixy = [1 0; 0 1; 0 -1; -1 0];
            nAdj = 4;
        case '8adj'
            ixy = [1 0; 0 1; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1];
            nAdj = 8;
    end

    % euclidean manhattan
    switch Model.distType
        case 'manhattan'
            edgeLength = 2;
        case 'euclidean'
            edgeLength = sqrt(2);
    end

    Successors = cell(Nodes.count, 2);
    Predecessors = cell(Nodes.count, 2);

    gv.robot = 0;
    GV = repmat(gv, Nodes.count, 1);

    for iNode = 1:Nodes.count

        if ~any(iNode == Obsts.nodeNumber)
            xNode = Nodes.cord(1, iNode);
            yNode = Nodes.cord(2, iNode);

            for iAdj = 1:nAdj
                ix = ixy(iAdj, 1);
                iy = ixy(iAdj, 2);
                newX = xNode + ix;
                newY = yNode + iy;

                % check if the Node is within array bound
                if (newX >= Map.xMin && newX <= Map.xMax) && (newY >= Map.yMin && newY <= Map.yMax)
                    newNodeNumber = iNode + ix + iy * (Map.nX);

                    if ~any(newNodeNumber == Obsts.nodeNumber)
                        Successors{iNode, 1} = [Successors{iNode, 1}, newNodeNumber];
                        Predecessors{newNodeNumber, 1} = [Predecessors{newNodeNumber, 1}, iNode];

                        if ix ~= 0 && iy ~= 0
                            cost = edgeLength;
                        else
                            cost = 1;
                        end

                        Successors{iNode, 2} = [Successors{iNode, 2}, cost];
                        Predecessors{newNodeNumber, 2} = [Predecessors{newNodeNumber, 1}, cost];
                    end

                end

            end

        else
            GV(iNode).Robots = -1;
        end

    end

    % G, RHS, km
    G = inf(Model.robotCount, Nodes.count);
    G = mat2cell(G, ones(1, Model.robotCount), Nodes.count);
    RHS = inf(Model.robotCount, Nodes.count);
    RHS = mat2cell(RHS, ones(1, Model.robotCount), Nodes.count);

    %% save Model
    Model.Nodes = Nodes;
    Model.Obsts = Obsts;
    Model.Map = Map;

    Model.Predecessors = Predecessors;
    Model.Successors = Successors;
    Model.RHS = RHS;
    Model.GV = GV;
    Model.G = G;

end
