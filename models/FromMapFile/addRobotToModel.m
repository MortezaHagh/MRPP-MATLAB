function Model = addRobotToModel(Model, scenarios)
    % add Robots data to Model

    v = randperm(size(scenarios, 1), Model.robotCount);
    sena = scenarios(v, :);

    %% robot data

    r.xs = 0;
    r.ys = 0;
    r.dir = 0;
    r.startNode = 0;
    r.targetNode = 0;
    Robots = repmat(r, Model.robotCount, 1);

    for iRobot = 1:Model.robotCount
        % start & goal - start & target coordinates
        Robots(iRobot).xs = sena(iRobot, 1);
        Robots(iRobot).ys = sena(iRobot, 2);
        Robots(iRobot).xt = sena(iRobot, 3);
        Robots(iRobot).yt = sena(iRobot, 4);

        Robots(iRobot).dir = deg2rad(randsample([0 90 180 270], 1));

        %  start & goal - node numbers
        Robots(iRobot).startNode = (Robots(iRobot).ys - Model.Map.yMin) * (Model.Map.nX) + Robots(iRobot).xs - Model.Map.xMin + 1;
        Robots(iRobot).targetNode = (Robots(iRobot).yt - Model.Map.yMin) * (Model.Map.nX) + Robots(iRobot).xt - Model.Map.xMin + 1;
    end

    %% save Model
    Model.Robots = Robots;

end
