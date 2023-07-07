function [GV, Open, Robots, Paths, nodesToUpdate] = initializeMRDSL(Model)
    % Initialize MRDSL MRPP problem.

    % G, RHS
    G = Model.G;
    GV = Model.GV;
    RHS = Model.RHS;

    % robots
    Robots = Model.Robots;
    robotCount = Model.robotCount;

    % topNode structure
    topNode.nodeNumber = 0;
    topNode.hCost = 0;
    topNode.key = 0;
    topNode.ind = 1;

    % Open structure
    oemp.count = 0;
    oemp.List = topNode;
    Open = repmat(oemp, robotCount, 1);

    % nodesToUpdate
    nodesToUpdate = cell(robotCount + 1, 1);

    % path structure
    p.nodeNumbers = [];
    p.smoothness = 0;
    p.coords = [];
    p.iRobot = 0;
    p.dirs = [];
    p.cost = 0;
    p.len = 0;
    p.x = [];
    p.y = [];
    Paths = repmat(p, robotCount, 1);

    % initialization
    for iRobot = 1:robotCount
        GV(Robots(iRobot).startNode).robot = iRobot;

        % set the starting node as the first node
        topNode.nodeNumber = Robots(iRobot).targetNode;
        topNode.hCost = calDistance(Robots(iRobot).xs, Robots(iRobot).ys, Robots(iRobot).xt, Robots(iRobot).yt, Model.distType);
        topNode.key = [topNode.hCost; 0];
        topNode.ind = 1;

        % insert start node in Open list
        Open(iRobot).count = 1;
        Open(iRobot).List(1) = topNode;

        % robots: G, RHS, path, sLast, Start, sGoal
        RHS{iRobot}(topNode.nodeNumber) = 0;
        Robots(iRobot).km = 0;
        Robots(iRobot).csp_flag = 1;
        Robots(iRobot).G = G{iRobot};
        Robots(iRobot).iRobot = iRobot;
        Robots(iRobot).RHS = RHS{iRobot};
        Robots(iRobot).Start.key = inf * [1; 1];
        Robots(iRobot).path = Robots(iRobot).startNode;
        Robots(iRobot).prevPath = Robots(iRobot).path;
        Robots(iRobot).sLast = Robots(iRobot).startNode;
        Robots(iRobot).sGoal = Robots(iRobot).targetNode;
        Robots(iRobot).Start.nodeNumber = Robots(iRobot).startNode;
        Robots(iRobot).updateLater = [];
    end

end
