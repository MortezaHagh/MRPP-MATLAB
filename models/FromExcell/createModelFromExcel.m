function Model = createModelFromExcel(Model)

    %% read data
    data = xlsread('Book1.xlsx', 'Sheet6');

    %% Algorithm
    % max_stall_count
    MSC = 40;

    %% Map
    Map.lim = data(1, 1);
    Map.xMin = 0;
    Map.xMax = data(1, 1);
    Map.yMin = 0;
    Map.yMax = data(2, 1);

    Map.nX = Map.xMax - Map.xMin + 1;
    Map.nY = Map.yMax - Map.yMin + 1;

    %% Robots data
    robotCount = data(1, 2);
    Model.robotCount = robotCount;

    r.dir = 0;
    r.xs = 0; r.ys = 0;
    r.xt = 0; r.yt = 0;
    r.targetNode = 0;
    r.startNode = 0;

    Robots = repmat(r, robotCount, 1);

    for iRobot = 1:robotCount
        j = iRobot + 6;
        Robots(iRobot).xs = data(1, j);
        Robots(iRobot).ys = data(2, j);
        Robots(iRobot).xt = data(3, j);
        Robots(iRobot).yt = data(4, j);
        Robots(iRobot).dir = data(5, j);
    end

    %  start & goal - node numbers
    for iRobot = 1:robotCount
        Robots(iRobot).startNode = (Robots(iRobot).ys - Map.yMin) * (Map.xMax - Map.xMin + 1) + Robots(iRobot).xs - Map.xMin + 1;
        Robots(iRobot).targetNode = (Robots(iRobot).yt - Map.yMin) * (Map.xMax - Map.xMin + 1) + Robots(iRobot).xt - Map.xMin + 1;
    end

    %% Obstacle

    % radius
    Obsts.r = 0.25;

    Obsts.count = data(1, 3);
    Obst2.count = data(1, 4);

    Obsts.x = zeros(1, Obsts.count);
    Obsts.y = zeros(1, Obsts.count);
    Obsts.nodeNumber = zeros(1, Obsts.count);

    for iObst = 1:Obsts.count
        Obsts.x(iObst) = data(iObst, 4);
        Obsts.y(iObst) = data(iObst, 5);
        Obsts.nodeNumber(iObst) = (Obsts.y(iObst) - Map.yMin) * (Map.xMax - Map.xMin + 1) + Obsts.x(iObst) - Map.xMin + 1;
    end

    Obst2.x = zeros(1, Obst2.count);
    Obst2.y = zeros(1, Obst2.count);
    Obst2.nodeNumber = zeros(1, Obst2.count);

    for iObst = 1:Obst2.count
        Obst2.x(iObst) = data(iObst, 4);
        Obst2.y(iObst) = data(iObst, 5);
        Obst2.nodeNumber(iObst) = (Obst2.y(iObst) - Map.yMin) * (Map.xMax - Map.xMin + 1) + Obst2.x(iObst) - Map.xMin + 1;
    end

    %% nodes & adj data
    iNode = 0;

    for iy = Map.yMin:Map.yMax

        for ix = Map.xMin:Map.xMax
            iNode = iNode + 1;
            Nodes.cord(1:2, iNode) = [ix, iy]'; % node coordinates
        end

    end

    Nodes.count = iNode;

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
                    newNodeNumber = iNode + ix + iy * (Map.xMax - Map.xMin + 1);

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
            GV(iNode).robot = -1;
        end

    end

    % G, RHS, km
    G = inf(robotCount, Nodes.count);
    G = mat2cell(G, ones(1, robotCount), Nodes.count);
    RHS = inf(robotCount, Nodes.count);
    RHS = mat2cell(RHS, ones(1, robotCount), Nodes.count);

    %% save Model
    Model.Nodes = Nodes;
    Model.Robots = Robots;
    Model.Obst2 = Obst2;
    Model.Obsts = Obsts;
    Model.Map = Map;

    Model.Predecessors = Predecessors;
    Model.Successors = Successors;
    Model.RHS = RHS;
    Model.GV = GV;
    Model.G = G;

end
