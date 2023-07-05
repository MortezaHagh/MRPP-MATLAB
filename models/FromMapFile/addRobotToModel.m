function Model = addRobotToModel(Model, scenarios)
% add Robot data to Model

v = randperm(size(scenarios,1), Model.robotCount);
sena = scenarios(v,:);

%% robot data

r.xs=0;
r.ys=0;
r.dir=0;
r.startNode = 0;
r.targetNode = 0;
Robot = repmat(r, Model.robotCount,1);


for iRobot=1:Model.robotCount
    % start & goal - start & target coordinates
    Robot(iRobot).xs = sena(iRobot,1);
    Robot(iRobot).ys = sena(iRobot,2);
    Robot(iRobot).xt = sena(iRobot,3);
    Robot(iRobot).yt = sena(iRobot,4);
    
    Robot(iRobot).dir = randsample([0 90 180 270],1);
    
    %  start & goal - node numbers
    Robot(iRobot).startNode = (Robot(iRobot).ys-Model.Map.yMin)*(Model.Map.nX)+Robot(iRobot).xs-Model.Map.xMin+1;
    Robot(iRobot).targetNode = (Robot(iRobot).yt-Model.Map.yMin)*(Model.Map.nX)+Robot(iRobot).xt-Model.Map.xMin+1;
end

%% save model
Model.Robot = Robot;

end
