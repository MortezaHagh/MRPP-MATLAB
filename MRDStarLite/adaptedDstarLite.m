function [Model, GV, Open, Robot, isComplete, nodesToUpdate] = adaptedDstarLite(Model, GV, Open, Robot, isComplete, nodesToUpdate)

    % update GV for previous path
    oldStartNode = Robot.Start.nodeNumber;

    for iNodeNum = Robot.prevPath

        if GV(iNodeNum).robot == Robot.iRobot
            GV(iNodeNum).robot = 0;
        end

    end

    nodesToUpdate2 = [];
    vRobot = [1:Robot.iRobot - 1, Robot.iRobot + 1:Model.robotCount + 1];

    for iRobot = vRobot

        if isempty(nodesToUpdate{iRobot}) || isempty(nodesToUpdate2)
            nodesToUpdate2 = [nodesToUpdate2, nodesToUpdate{iRobot}]; %#ok
        else
            nodesToUpdate2 = union(nodesToUpdate2, nodesToUpdate{iRobot});
        end

    end

    % check for update in edge costs (obstacles)
    if ~isempty(nodesToUpdate2)
        [Open, Robot] = updateVertex(Open, GV, Robot, nodesToUpdate2, Model);
    end

    %% computeShortestPath and check for conditions

    % compute shortest path
    if ~isempty(Open.List)
        [Open, Robot] = computeShortestPath(Open, GV, Robot, Model);
    end

    if Robot.csp_flag == 0
        disp([num2str(Robot.iRobot) ' csp_flag==0 !']);
        nodes = oldStartNode;
        GV(oldStartNode).robot = Robot.iRobot;
        Robot.path(end + 1) = oldStartNode;
        [nodesToUpdate{Robot.iRobot}, Robot] = updateData(nodes, Robot, Model);
        return
    end

    % move robot to next node (empty or reserved by robot itself)
    sucNodesCandid = Model.Successors{Robot.Start.nodeNumber, 1};
    ocpRobotIds = [GV(sucNodesCandid).robot];
    ocpRobotIds = ocpRobotIds == Robot.iRobot | ocpRobotIds == 0;
    sucNodes = sucNodesCandid(ocpRobotIds);

    % new start node
    [~, sortedInds] = sortrows([Robot.G(sucNodes) + Model.Successors{Robot.Start.nodeNumber, 2}(ocpRobotIds); rand(1, numel(sucNodes))]');
    newStartNode = sucNodes(sortedInds(1));

    % new start node and move
    Robot.Start.nodeNumber = newStartNode;
    Robot.path(end + 1) = Robot.Start.nodeNumber;
    xy_ss = Model.Nodes.cord(:, Robot.sLast);
    xy_sl = Model.Nodes.cord(:, Robot.Start.nodeNumber);
    Robot.km = Robot.km +calDistance(xy_sl(1), xy_sl(2), xy_ss(1), xy_ss(2), Model.distType);
    Robot.sLast = Robot.Start.nodeNumber;

    if Robot.Start.nodeNumber ~= Robot.sGoal

        % find optimapl path
        s = Robot.Start.nodeNumber;
        nodes = s;

        while s ~= Robot.sGoal && numel(nodes) < Model.occLength
            sucNodesCandid = Model.Successors{s, 1};
            ocpRobotIds = [GV(sucNodesCandid).robot];
            ocpRobotIds = find(ocpRobotIds == Robot.iRobot | ocpRobotIds == 0);
            sucNodes = sucNodesCandid(ocpRobotIds);
            [sucNodes, indsucNodes] = setdiff(sucNodes, nodes);
            indsucNodes = ocpRobotIds(indsucNodes);

            if isempty(sucNodes)
                disp([num2str(Robot.iRobot) ' sucNodes is empty ########2!']);
                break
            end

            [sortCost, sortedInds] = sortrows([Robot.G(sucNodes) + Model.Successors{s, 2}(indsucNodes); rand(1, numel(sucNodes))]');
            newStartNode = sucNodes(sortedInds(1));

            if any(sucNodesCandid == Robot.sGoal) || sortCost(1) == inf

                if Robot.sGoal ~= newStartNode || sortCost(1) == inf
                    disp([num2str(Robot.iRobot) ' Robot.sGoal~=newStartNode or sortCost(1)==inf ######2 !']);
                    break
                end

            end

            s = sucNodes(sortedInds(1));
            nodes(end + 1) = s; %#ok
        end

    else
        isComplete = 1;
        nodes = Robot.sGoal;
    end

    % update GV of new path
    for s = nodes
        GV(s).robot = Robot.iRobot;
    end

    [nodesToUpdate{Robot.iRobot}, Robot] = updateData(nodes, Robot, Model);

end

function [nodesToUpdate, Robot] = updateData(nodes, Robot, Model)

    % nodes to update for other robots
    nodesToUpdate3 = setxor(nodes, Robot.prevPath);
    Robot.prevPath = nodes;
    neighbors = [];

    for iNode = nodesToUpdate3
        neighbors = [neighbors Model.Predecessors{iNode, 1}]; %#ok
    end

    nodesToUpdate3 = union(nodesToUpdate3, neighbors);
    nodesToUpdate = nodesToUpdate3;

end
