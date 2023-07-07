function isFeasible = checkFeasibility(Model)

    addpath('..\common');
    addpath('..\models');

    Robots = Model.Robots;
    isFeasible = true;

    % not considering other robots
    for iRobot = 1:Model.robotCount
        Model.Robots = Robots(iRobot);

        try
            myAStar(Model);
        catch ME
            isFeasible = false;

            switch ME.identifier
                case 'Astar:noPath'
                    disp(' No Path, case 1 !');
                    return
                otherwise
                    rethrow(ME)
                    disp(ME.message);
            end

        end

    end

    % not considering other robots
    Model.Obsts.nodeNumber = [Model.Obsts.nodeNumber [Robots.targetNode]];
    Model.Obsts.x = [Model.Obsts.x [Robots.xt]];
    Model.Obsts.y = [Model.Obsts.y [Robots.yt]];

    isFeasible2 = 1;

    for iRobot = 1:Model.robotCount
        Model = Model;
        Model.Robots = Robots(iRobot);
        Model.Obsts.nodeNumber(Model.Obsts.count + iRobot) = [];
        Model.Obsts.x(Model.Obsts.count + iRobot) = [];
        Model.Obsts.y(Model.Obsts.count + iRobot) = [];
        Model.Obsts.count = Model.Obsts.count - 1 + Model.robotCount;

        try
            myAStar(Model);
        catch ME
            isFeasible = false;

            switch ME.identifier
                case 'Astar:noPath'
                    disp(' No Path, case 2 !');
                    return
                otherwise
                    rethrow(ME)
                    disp(ME.message);
            end

        end

    end

end
