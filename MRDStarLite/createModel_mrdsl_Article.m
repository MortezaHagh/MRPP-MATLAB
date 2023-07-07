function Model = createModel_mrdsl_Article(Model)
    % Create Complete Model for MRPP

    %% Map Size
    Map.lim = 9;
    Map.xMin = 1;
    Map.xMax = Map.lim;
    Map.yMin = 1;
    Map.yMax = Map.lim;

    Map.nX = Map.xMax - Map.xMin + 1;
    Map.nY = Map.yMax - Map.yMin + 1;

    %% robots data
    robotCount = 3;
    Model.robotCount = robotCount;

    % dir: direction
    Robot(1).dir = 0;
    Robot(2).dir = 0;
    Robot(3).dir = 0;
    % Robot(4).dir = deg2rad(randsample([0 90 180 270], 1));

    % start & goal
    Robot(1).xs = 4; Robot(1).ys = 3;
    Robot(1).xt = 8; Robot(1).yt = 8;

    Robot(2).xs = 8; Robot(2).ys = 2;
    Robot(2).xt = 3; Robot(2).yt = 8;

    Robot(3).xs = 2; Robot(3).ys = 5;
    Robot(3).xt = 9; Robot(3).yt = 5;

    % Robot(4).xs = 1;   Robot(4).ys = 1;
    % Robot(4).xt = 20;   Robot(4).yt = 20;

    %  start & goal - node numbers
    for iRobot = 1:robotCount
        Robot(iRobot).startNode = (Robot(iRobot).ys - Map.yMin) * (Map.nX) + Robot(iRobot).xs + abs(Map.xMin - 1);
        Robot(iRobot).targetNode = (Robot(iRobot).yt - Map.yMin) * (Map.nX) + Robot(iRobot).xt + abs(Map.xMin - 1);
    end

    %% Obstacle

    % radius
    Obst.r = 0.25;

    % Obstacle coordinates
    xc1 = [8, 5, 4, 6, 8];
    yc1 = [5, 7, 7, 2, 6];

    Obst.x = xc1;
    Obst.y = yc1;

    Obst.count = length(Obst.x);

    % obstacle node numbers
    Obst.nodeNumber = zeros(1, Obst.count);

    for iObst = 1:Obst.count
        Obst.nodeNumber(iObst) = (Obst.y(iObst) - Map.yMin) * (Map.nX) + Obst.x(iObst) + abs(Map.xMin - 1);
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

    gv.adj = [];
    gv.robot = 0;
    GV = repmat(gv, Nodes.count, 1);

    for iNode = 1:Nodes.count

        if ~any(iNode == Obst.nodeNumber)
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

                    if ~any(newNodeNumber == Obst.nodeNumber)
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

    %% save model
    Model.Nodes = Nodes;
    Model.Robot = Robot;
    Model.Obst = Obst;
    Model.Map = Map;

    Model.Predecessors = Predecessors;
    Model.Successors = Successors;
    Model.RHS = RHS;
    Model.GV = GV;
    Model.G = G;

    %% plot model
    % plotModelMulti(model);

end
