function Model = createModelRand(Model, nObst, nRobot)
    % Create Complete Model for MRPP - Random

    %% Map
    Map.lim = 20;
    Map.xMin = 1;
    Map.xMax = 30;
    Map.yMin = 1;
    Map.yMax = 20;

    Map.nX = Map.xMax - Map.xMin + 1;
    Map.nY = Map.yMax - Map.yMin + 1;

    %% Generate Random Arrangments
    Obsts.count = nObst;
    robotCount = nRobot;
    Model.robotCount = robotCount;

    nNodes = (Map.nX * Map.nY);

    X = zeros(Map.nX, Map.nY);
    Y = zeros(Map.nX, Map.nY);

    for iRobot = 1:Map.nX

        for iy = 1:Map.nY
            X(iRobot, iy) = Map.xMin + iRobot - 1;
            Y(iRobot, iy) = Map.yMin + iy - 1;
        end

    end

    aRand = randsample(nNodes, 2 * robotCount + Obsts.count);

    randStart = aRand(1:robotCount);
    xStart = X(randStart);
    yStart = Y(randStart);

    randTarget = aRand(robotCount + 1:2 * robotCount);
    xTarget = X(randTarget);
    yTarget = Y(randTarget);

    randObstacles = aRand(2 * robotCount + 1:2 * robotCount + Obsts.count);
    Obsts.x = X(randObstacles)';
    Obsts.y = Y(randObstacles)';

    % obstacle node numbers
    Obsts.nodeNumber = zeros(1, Obsts.count);

    for iObst = 1:Obsts.count
        Obsts.nodeNumber(iObst) = (Obsts.y(iObst) - Map.yMin) * (Map.nX) + Obsts.x(iObst) - Map.xMin + 1;
    end

    Obsts.r = 0.25;

    %% robots data
    r.dir = 0;
    r.xs = 0;
    r.ys = 0;
    r.xt = 0;
    r.yt = 0;
    r.targetNode = 0;
    r.startNode = 0;

    Robot = repmat(r, robotCount, 1);

    for iRobot = 1:robotCount
        Robot(iRobot).xs = xStart(iRobot);
        Robot(iRobot).ys = yStart(iRobot);
        Robot(iRobot).xt = xTarget(iRobot);
        Robot(iRobot).yt = yTarget(iRobot);
        Robot(iRobot).dir = 0; % deg2rad(randsample([0 90 180 270], 1));
    end

    %  start & goal - node numbers
    for iRobot = 1:robotCount
        Robot(iRobot).startNode = (Robot(iRobot).ys - Map.yMin) * (Map.nX) + Robot(iRobot).xs + abs(Map.xMin - 1);
        Robot(iRobot).targetNode = (Robot(iRobot).yt - Map.yMin) * (Map.nX) + Robot(iRobot).xt + abs(Map.xMin - 1);
    end

    %% nodes & adj data
    iNode = 0;

    for iy = Map.yMin:Map.yMax

        for ix = Map.xMin:Map.xMax
            iNode = iNode + 1;
            Nodes.cord(1:2, iNode) = [ix, iy]'; % node coordinates
            Nodes.number(1, iNode) = iNode; % node number
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
    Model.Robots = Robot;
    Model.Obsts = Obsts;
    Model.Map = Map;

    Model.Predecessors = Predecessors;
    Model.Successors = Successors;
    Model.RHS = RHS;
    Model.GV = GV;
    Model.G = G;

end
