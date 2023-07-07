function Paths = MRPP_3(Model)
    % with stall node & time
    % calculate path just after each reach, with stall node *

    % Initialization and Parameters
    [Closed, Open, Topnodes, Robots, Paths, closedInit] = initializationMh(Model);
    robotCount = Model.robotCount;
    oTopInd = zeros(robotCount, 1); % openTopInd
    npCount = zeros(robotCount, 1); % no_path_count

    %% Start Algorithm

    % defining flags
    isPath = ones(robotCount, 1); % was pp successful?
    MissionFlag = true(robotCount, 1); % is pp for robot ended?
    % path_complete_flag = MissionFlag;        % was optimal path calculated?

    robot_list = 1:robotCount;
    success = false;
    ttt = 0;

    while ~success && ttt < 5

        while any(MissionFlag)
            priority_list = priority(Robots, robot_list, Model, [Topnodes.nodeNumber]);

            for nr = priority_list
                %     for nr=1:robotCount

                % update mission flag
                %         MissionFlag(nr) = (Topnodes(nr).nodeNumber~=Robots(nr).targetNode && isPath(nr) == 1);
                if MissionFlag(nr)

                    % finding neighbors (successors)
                    neighbors = expand(Topnodes(nr), Closed(nr), Model, nr);

                    % if strcmp(Model.adjType, '4adj')
                    %     neighbors = neighbors4(Topnodes(nr), Closed(nr), Model, nr);
                    % elseif strcmp(Model.adjType, '8adj')
                    %     neighbors = neighbors8(Topnodes(nr), Closed(nr), Model, nr);
                    % end

                    % update or extend Open list with the successor nodes
                    Open(nr) = updateOpen(Open(nr), neighbors);

                    % select new Top Node
                    oTopInd(nr) = selectTopNode_1(Open, Robots(nr).targetNode, Topnodes(nr).dir, nr);

                    % if no path exists to the Target -> try stall nodes
                    if oTopInd(nr) == -1 && npCount(nr) < Model.msc
                        Closed(nr) = closedInit(nr);
                        [oTopInd(nr), npCount(nr), Open] = addStallNodes(npCount(nr), Open ...
                            , Robots(nr).targetNode, Topnodes(nr).dir, nr, Model.msc);
                    end

                    % update Open & close with new topNode
                    if oTopInd(nr) ~= -1
                        Open(nr).List(oTopInd(nr)).visited = 1;
                        Topnodes(nr) = Open(nr).List(oTopInd(nr));
                        Closed(nr).count = Closed(nr).count + 1;
                        Closed(nr).nodeNumbers(end + 1) = Topnodes(nr).nodeNumber;
                    else
                        % if after trying stall nodes, no path found, robot stays at start node
                        isPath(nr) = 0;
                        MissionFlag(nr) = false;
                        disp(['No Path for robot ' num2str(nr) '!'])
                    end

                    if Topnodes(nr).nodeNumber == Robots(nr).targetNode
                        MissionFlag(nr) = false;
                    end

                    if MissionFlag(nr) == false
                        % Optimal Path for robot nr & update Open(nr)
                        Paths(nr) = optimalPath_1(Model, Open(nr), isPath, nr);
                        Open(nr) = finalOpen(Paths(nr));
                    end

                end

            end

        end

        [t, robo] = collisionsCheck2(Paths, robotCount);

        if robo == 0
            success = true;
        else
            MissionFlag(robo) = true;
            priority_list = robo;
            ttt = ttt + 1;
            % Closed: put all obstacles on the Closed list
            Closed(robo).count = Model.obstCount;
            Closed(robo).nodeNumbers = Model.obstNode;

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

            % insert start node in Open list
            Open(robo).count = 1;
            Open(robo).List = topNode;

            % add last node (hear start node) to Closed
            Closed(robo).count = Closed(robo).count + 1;
            Closed(robo).nodeNumbers(end + 1) = topNode.nodeNumber;
        end

    end

end
