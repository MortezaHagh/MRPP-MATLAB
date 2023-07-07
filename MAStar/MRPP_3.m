function paths = MRPP_3(Model)
    % with stall node & time
    % calculate path just after each reach, with stall node *

    % Initialization and Parameters
    [closed, open, topnodes, robots, paths, closed_init] = initialization_mh(Model);
    robot_count = Model.robot_count;
    ot_ind = zeros(robot_count, 1); % open_top_ind
    np_count = zeros(robot_count, 1); % no_path_count

    %% Start Algorithm

    % defining flags
    isPath = ones(robot_count, 1); % was pp successful?
    MissionFlag = true(robot_count, 1); % is pp for robot ended?
    % path_complete_flag = MissionFlag;        % was optimal path calculated?

    robot_list = 1:robot_count;
    success = false;
    ttt = 0;

    while ~success && ttt < 5

        while any(MissionFlag)
            priority_list = priority(robots, robot_list, Model, [topnodes.nodeNumber]);

            for nr = priority_list
                %     for nr=1:robot_count

                % update mission flag
                %         MissionFlag(nr) = (topnodes(nr).nodeNumber~=robots(nr).targetNode && isPath(nr) == 1);
                if MissionFlag(nr)

                    % finding neighbors (successors)
                    if strcmp(Model.adjType, '4adj')
                        neighbors = neighbors4(topnodes(nr), closed(nr), Model, nr);
                    elseif strcmp(Model.adjType, '8adj')
                        neighbors = neighbors8(topnodes(nr), closed(nr), Model, nr);
                    end

                    % update or extend open list with the successor nodes
                    open(nr) = updateOpen(open(nr), neighbors);

                    % select new Top Node
                    ot_ind(nr) = selectTopNode_1(open, robots(nr).targetNode, topnodes(nr).dir, nr);

                    % if no path exists to the Target -> try stall nodes
                    if ot_ind(nr) == -1 && np_count(nr) < Model.msc
                        closed(nr) = closed_init(nr);
                        [ot_ind(nr), np_count(nr), open] = addStallNodes(np_count(nr), open ...
                            , robots(nr).targetNode, topnodes(nr).dir, nr, Model.msc);
                    end

                    % update open & close with new topNode
                    if ot_ind(nr) ~= -1
                        open(nr).List(ot_ind(nr)).visited = 1;
                        topnodes(nr) = open(nr).List(ot_ind(nr));
                        closed(nr).count = closed(nr).count + 1;
                        closed(nr).nodeNumbers(end + 1) = topnodes(nr).nodeNumber;
                    else
                        % if after trying stall nodes, no path found, robot stays at start node
                        isPath(nr) = 0;
                        MissionFlag(nr) = false;
                        disp(['No Path for robot ' num2str(nr) '!'])
                    end

                    if topnodes(nr).nodeNumber == robots(nr).targetNode
                        MissionFlag(nr) = false;
                    end

                    if MissionFlag(nr) == false
                        % Optimal Path for robot nr & update open(nr)
                        paths(nr) = optimalPath_1(Model, open(nr), isPath, nr);
                        open(nr) = finalOpen(paths(nr));
                    end

                end

            end

        end

        [t, robo] = collisionsCheck2(paths, robot_count);

        if robo == 0
            success = true;
        else
            MissionFlag(robo) = true;
            priority_list = robo;
            ttt = ttt + 1;
            % closed: put all obstacles on the Closed list
            closed(robo).count = Model.numOfObs;
            closed(robo).nodeNumbers = Model.obstNode;

            % set the starting node (topNode) as the first node in Open
            topNode.visited = 1;
            topNode.nodeNumber = Model.Robots(robo).startNode;
            topNode.pNode = Model.Robots(robo).startNode;
            topNode.dir = Model.Robots(robo).dir;
            topNode.gCost = 0;
            hCost = calDistance(Model.Robots(robo).xs, Model.Robots(robo).ys, Model.Robots(robo).xt, Model.Robots(robo).yt, Model.distType) * 2;
            topNode.fCost = topNode.gCost + hCost;
            topNode.time = 0;
            topNode.tag = 1;

            % insert start node in open list
            open(robo).count = 1;
            open(robo).List = topNode;

            % add last node (hear start node) to Closed
            closed(robo).count = closed(robo).count + 1;
            closed(robo).nodeNumbers(end + 1) = topNode.nodeNumber;
        end

    end

end
