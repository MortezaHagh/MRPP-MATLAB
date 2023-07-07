function [Open, Robot] = updateVertex(Open, GV, Robot, nodesForUpdate, Model)

    for nodeNumber = nodesForUpdate

        if nodeNumber ~= Robot.targetNode

            if (GV(nodeNumber).robot == 0 || GV(nodeNumber).robot == Robot.iRobot)
                succNodes = Model.Successors{nodeNumber, 1};
                ind = [GV(succNodes).robot] == Robot.iRobot | [GV(succNodes).robot] == 0;
                succNodes = succNodes(ind);

                if ~isempty(succNodes)
                    [val_minG, ~] = min(Robot.G(succNodes) + Model.Successors{nodeNumber, 2}(ind));
                    Robot.RHS(nodeNumber) = val_minG;
                else
                    Robot.RHS(nodeNumber) = inf;
                end

            else
                Robot.RHS(nodeNumber) = inf;
            end

        end

        check = nodeNumber == [Open.List.nodeNumber];

        if any(check)
            Open.List(check) = [];
            Open.count = Open.count - 1;
        end

        if Robot.G(nodeNumber) ~= Robot.RHS(nodeNumber)
            Open.count = Open.count + 1;
            op.nodeNumber = nodeNumber;
            nodeXY = Model.Nodes.cord(:, nodeNumber);
            robotXY = Model.Nodes.cord(:, Robot.Start.nodeNumber);
            op.hCost = calDistance(robotXY(1), robotXY(2), nodeXY(1), nodeXY(2), Model.distType);
            op.key = min(Robot.G(nodeNumber), Robot.RHS(nodeNumber)) + [op.hCost + Robot.km; 0];
            op.ind = Open.count;
            Open.List(op.ind) = op;
        end

    end

end
