function isFeasible = checkFeasibility(Model)

    addpath('..\common');
    addpath('..\models');

    Robots = Model.Robot;
    isFeasible = true;

    % not considering other robots
    for iRobot = 1:Model.robotCount
        Model.Robot = Robots(iRobot);

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
    Model.Obst.nodeNumber = [Model.Obst.nodeNumber [Robots.targetNode]];
    Model.Obst.x = [Model.Obst.x [Robots.xt]];
    Model.Obst.y = [Model.Obst.y [Robots.yt]];

    isFeasible2 = 1;

    for iRobot = 1:Model.robotCount
        Model = Model;
        Model.Robot = Robots(iRobot);
        Model.Obst.nodeNumber(Model.Obst.count + iRobot) = [];
        Model.Obst.x(Model.Obst.count + iRobot) = [];
        Model.Obst.y(Model.Obst.count + iRobot) = [];
        Model.Obst.count = Model.Obst.count - 1 + Model.robotCount;

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
