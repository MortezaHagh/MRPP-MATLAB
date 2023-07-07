function Model = createModel_mrdsl_2(Model)
    % Create Complete Model for MRPP

    %% Map
    Map.lim = 28;
    Map.xMin = 1;
    Map.xMax = Map.lim;
    Map.yMin = 1;
    Map.yMax = Map.lim;

    Map.nX = Map.xMax - Map.xMin + 1;
    Map.nY = Map.yMax - Map.yMin + 1;

    %% robots data
    robotCount = 29;
    Model.robotCount = robotCount;

    % dir: direction
    Robot(1).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(2).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(3).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(4).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(5).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(6).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(7).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(8).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(9).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(10).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(11).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(12).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(13).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(14).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(15).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(16).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(17).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(18).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(19).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(20).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(21).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(22).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(23).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(24).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(25).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(26).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(27).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(28).dir = deg2rad(randsample([0 90 180 270], 1));
    Robot(29).dir = deg2rad(randsample([0 90 180 270], 1));

    % start & goal
    Robot(1).xs = 1; Robot(1).ys = 4;
    Robot(1).xt = 21; Robot(1).yt = 14;

    Robot(2).xs = 25; Robot(2).ys = 4;
    Robot(2).xt = 4; Robot(2).yt = 14;

    Robot(3).xs = 5; Robot(3).ys = 7;
    Robot(3).xt = 1; Robot(3).yt = 12;

    Robot(4).xs = 20; Robot(4).ys = 1;
    Robot(4).xt = 5; Robot(4).yt = 23;

    Robot(5).xs = 5; Robot(5).ys = 1;
    Robot(5).xt = 21; Robot(5).yt = 23;

    Robot(6).xs = 12; Robot(6).ys = 1;
    Robot(6).xt = 12; Robot(6).yt = 25;

    Robot(7).xs = 1; Robot(7).ys = 14;
    Robot(7).xt = 12; Robot(7).yt = 19;

    Robot(8).xs = 2; Robot(8).ys = 19;
    Robot(8).xt = 24; Robot(8).yt = 19;

    Robot(9).xs = 4; Robot(9).ys = 19;
    Robot(9).xt = 21; Robot(9).yt = 19;

    Robot(10).xs = 1; Robot(10).ys = 16;
    Robot(10).xt = 25; Robot(10).yt = 16;

    Robot(11).xs = 8; Robot(11).ys = 1;
    Robot(11).xt = 8; Robot(11).yt = 25;

    Robot(12).xs = 17; Robot(12).ys = 1;
    Robot(12).xt = 17; Robot(12).yt = 24;

    Robot(13).xs = 1; Robot(13).ys = 11;
    Robot(13).xt = 24; Robot(13).yt = 14;

    Robot(14).xs = 15; Robot(14).ys = 4;
    Robot(14).xt = 6; Robot(14).yt = 19;

    Robot(15).xs = 2; Robot(15).ys = 1;
    Robot(15).xt = 25; Robot(15).yt = 23;

    Robot(16).xs = 24; Robot(16).ys = 1;
    Robot(16).xt = 1; Robot(16).yt = 24;

    Robot(17).xs = 6; Robot(17).ys = 9;
    Robot(17).xt = 17; Robot(17).yt = 14;

    Robot(18).xs = 21; Robot(18).ys = 4;
    Robot(18).xt = 6; Robot(18).yt = 14;

    Robot(19).xs = 25; Robot(19).ys = 18;
    Robot(19).xt = 10; Robot(19).yt = 14;

    Robot(20).xs = 10; Robot(20).ys = 1;
    Robot(20).xt = 10; Robot(20).yt = 19;

    Robot(21).xs = 4; Robot(21).ys = 1;
    Robot(21).xt = 4; Robot(21).yt = 25;

    Robot(22).xs = 10; Robot(22).ys = 9;
    Robot(22).xt = 24; Robot(22).yt = 9;

    Robot(23).xs = 17; Robot(23).ys = 4;
    Robot(23).xt = 15; Robot(23).yt = 23;

    Robot(24).xs = 21; Robot(24).ys = 9;
    Robot(24).xt = 17; Robot(24).yt = 19;

    Robot(25).xs = 13; Robot(25).ys = 1;
    Robot(25).xt = 13; Robot(25).yt = 23;

    Robot(26).xs = 17; Robot(26).ys = 9;
    Robot(26).xt = 15; Robot(26).yt = 19;

    Robot(27).xs = 19; Robot(27).ys = 9;
    Robot(27).xt = 11; Robot(27).yt = 23;

    Robot(28).xs = 22; Robot(28).ys = 1;
    Robot(28).xt = 6; Robot(28).yt = 25;

    Robot(29).xs = 1; Robot(29).ys = 21;
    Robot(29).xt = 23; Robot(29).yt = 11;

    %  start & goal - node numbers
    for iRobot = 1:robotCount
        Robot(iRobot).startNode = (Robot(iRobot).ys - Map.yMin) * (Map.nX) + Robot(iRobot).xs - Map.xMin + 1;
        Robot(iRobot).targetNode = (Robot(iRobot).yt - Map.yMin) * (Map.nX) + Robot(iRobot).xt - Map.xMin + 1;
    end

    %% Obstacle

    % radius
    Obsts.r = 0.25;

    % Obstacle coordinates
    xc1 = [4 4 4 6 6 6 8 8 8 10 10 10 12 12 12];
    yc1 = [4 5 6 4 5 6 4 5 6 4 5 6 4 5 6];

    xc2 = [xc1 xc1 xc1 xc1];
    yc2 = [yc1 yc1 + 6 yc1 + 12 yc1 + 18];

    xc3 = xc2 + 12;
    yc3 = yc2;

    Obsts.x = [xc2 xc3];
    Obsts.y = [yc2 yc3];

    Obsts.count = length(Obsts.x);

    % obstacle node numbers
    Obsts.nodeNumber = zeros(1, Obsts.count);

    for iObst = 1:Obsts.count
        Obsts.nodeNumber(iObst) = (Obsts.y(iObst) - Map.yMin) * (Map.nX) + Obsts.x(iObst) - Map.xMin + 1;
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

    %% plot Model
    % plotModel(Model);

end
