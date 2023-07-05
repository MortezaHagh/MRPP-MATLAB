function isFeasible = checkFeasibility(Model)

addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB\SRPP');
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB\SRPP\01-SRPP-Astar');

Robots = Model.Robot;
isFeasible =true;

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
                %disp(ME.message);
        end
    end
end


% not considering other robots
Model.Obst.nodeNumber = [Model.Obst.nodeNumber [Robots.targetNode]];
Model.Obst.x = [Model.Obst.x [Robots.xt]];
Model.Obst.y = [Model.Obst.y [Robots.yt]];

for iRobot = 1:Model.robotCount
    model = Model;
    model.Robot = Robots(iRobot);
    model.Obst.nodeNumber(model.Obst.count+iRobot)=[];
    model.Obst.x(model.Obst.count+iRobot)=[];
    model.Obst.y(model.Obst.count+iRobot)=[];
    model.Obst.count = model.Obst.count-1+Model.robotCount;
    
    try
        myAStar(model);
    catch ME
        isFeasible = false;
        switch ME.identifier
            case 'Astar:noPath'
                disp(' No Path, case 2 !');
                return
            otherwise
                rethrow(ME)
                % disp(ME.message);
        end
    end
end

end