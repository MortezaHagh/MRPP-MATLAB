function [Model, Robots, Paths] = myMRDstarLite(Model)


% initialization
[GV, Open, Robots, Paths, nodesToUpdate] = initializeMRDSL(Model);

k=1;
t=1;
robotCount = Model.robotCount;
robotList = 1:robotCount;
isPathComplete = zeros(robotCount,1);

%% multirobot D*Lite PP
while any(isPathComplete==0)
    
    % prioritize_robot_list
    priorityList = calculatePriority(Robots, robotList, Model);
    for iRobot = priorityList
        
        switch Model.expandMethod
            case 'random'
                ppFunction  = @ adaptedDstarLite;
            case 'heading'
                ppFunction  = @ adaptedDstarLite_dir;
        end
        
        % adaptedDstarLite, adaptedDstarLite_dir
        [Model, GV, Open(iRobot), Robots(iRobot), isPathComplete(iRobot), nodesToUpdate] =  ...
            ppFunction(Model, GV, Open(iRobot), Robots(iRobot),...
            isPathComplete(iRobot), nodesToUpdate);
        
        % update Robot list
        if isPathComplete(iRobot)==1
            robotList(robotList==iRobot)=[];
        end
        
    end
    
    % failure check
    isFailed = failureCheck(Robots, robotList);
    if isFailed
        if k==2
            disp(' **** Failed to plan paths for all robots! ****');
            break
        else
            k=k+1;
        end
    else
        k=0;
    end
    
    % update data based on the environmental changes
    %     t=t+1;
    %     [GV, Model, nodesToUpdate] = updateObstacles(nodesToUpdate, GV, Model, t);
    
end

%% optimal paths coordinations, nodes, directions

for iRobot=1:robotCount
    Paths(iRobot).iRobot = iRobot;
    Paths(iRobot).nodeNumbers = Robots(iRobot).path;
    Paths(iRobot).coords = nodes2coords(Paths(iRobot).nodeNumbers, Model);
    Paths(iRobot).dirs = nodes2dirs(Paths(iRobot).nodeNumbers, Model);
end

% % update model
% new_obst_xy = Model.Nodes.cord(:,3);
% Model.Obst.x = [Model.Obst.x, new_obst_xy(1,:)];
% Model.Obst.y = [Model.Obst.y, new_obst_xy(2,:)];

end
