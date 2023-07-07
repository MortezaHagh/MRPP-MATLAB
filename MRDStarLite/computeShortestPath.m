function [Open, Robot] = computeShortestPath(Open, GV, Robot, Model)

    csp_flag = 1;

    % Start
    Start = Robot.Start;

    % select top key
    TopNode = topKey(Open);

    % update start_key
    Start.nodeNumber = Robot.Start.nodeNumber;
    Start.key = min(Robot.G(Start.nodeNumber), Robot.RHS(Start.nodeNumber)) * [1; 1] + [Robot.km; 0];

    while (compareKeys(TopNode.key, Start.key) || Robot.RHS(Start.nodeNumber) ~= Robot.G(Start.nodeNumber))
        csp_flag = 1;
        k_old = TopNode.key;
        k_new = min(Robot.G(TopNode.nodeNumber), Robot.RHS(TopNode.nodeNumber)) + [TopNode.hCost + Robot.km; 0];

        % remove topkey from open
        Open.List(TopNode.ind) = [];
        Open.count = Open.count - 1;

        % update vertex
        nodesForUpdate = Model.Predecessors{TopNode.nodeNumber, 1};

        if compareKeys(k_old, k_new)
            Open.List(end + 1) = TopNode;
            Open.List(end).key = k_new;
            Open.count = Open.count + 1;
        else

            if Robot.G(TopNode.nodeNumber) > Robot.RHS(TopNode.nodeNumber)
                Robot.G(TopNode.nodeNumber) = Robot.RHS(TopNode.nodeNumber);
            else
                Robot.G(TopNode.nodeNumber) = inf;
                nodesForUpdate(end + 1) = TopNode.nodeNumber; %#ok
            end

            [Open, Robot] = updateVertex(Open, GV, Robot, nodesForUpdate, Model);
        end

        if TopNode.nodeNumber == Robot.Start.nodeNumber && Robot.RHS(Robot.Start.nodeNumber) == inf
            disp('   Robot Start.nodeNumber RHS Inf ********');
            csp_flag = 0;
            break
        end

        % check if Open List is empty
        if isempty(Open.List)
            break
        end

        % select top key
        TopNode = topKey(Open);

        if TopNode.key(2) == inf
            disp('===== inf cost top node!') % %to remove
            csp_flag = 0;
            break
        end

        % update start_key
        Start.key = min(Robot.G(Start.nodeNumber), Robot.RHS(Start.nodeNumber)) * [1; 1] + [Robot.km; 0];
    end

    if Robot.RHS(Start.nodeNumber) == inf
        disp('===== Open List is empty!')
        csp_flag = 0;
    end

    Robot.csp_flag = csp_flag;
end
