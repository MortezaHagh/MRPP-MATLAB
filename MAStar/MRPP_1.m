function paths = MRPP_1(Model)
    % with stall node & time

    % Initialization and Parameters
    [closed, open, topnodes, robo, paths] = initializationMh(Model);

    robotCount = Model.robotCount;
    ot_ind = zeros(robotCount, 1); % open_top_ind
    np_count = zeros(robotCount, 1); % no_path_count

    %% Start Algorithm

    % flags
    isPath = ones(robotCount, 1); % was pp successful?
    MissionFlag = true(robotCount, 1); % is pp for robot ended?

    while any(MissionFlag)

        for nr = 1:robotCount

            % update mission flag
            MissionFlag(nr) = (topnodes(nr).nodeNumber ~= robo(nr).targetNode && isPath(nr) == 1);

            if MissionFlag(nr)

                % finding neighbors (successors)
                neighbors = expand(topnodes(nr), closed(nr), Model, nr);

                % if strcmp(Model.adjType, '4adj')
                %     neighbors = neighbors4(topnodes(nr), closed(nr), Model, nr);
                % elseif strcmp(Model.adjType, '8adj')
                %     neighbors = neighbors8(topnodes(nr), closed(nr), Model, nr);
                % end

                % update or extend open list with the successor nodes
                open(nr) = updateOpen(open(nr), neighbors);

                % select new Top Node
                ot_ind(nr) = selectTopNode_1(open, robo(nr).targetNode, topnodes(nr).dir, nr);

                % if no path exists to the Target -> try stall nodes
                if ot_ind(nr) == -1 && np_count(nr) < Model.msc
                    [ot_ind(nr), np_count(nr), open] = addStallNodes(np_count(nr), open ...
                        , robo(nr).targetNode, topnodes(nr).dir, nr, Model.msc);
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
                    disp(['No Path for robot ' num2str(nr) '!'])
                end

            end

        end

    end

    %% Optimal Path
    for nr = 1:robotCount
        paths(nr) = optimalPath_1(Model, open(nr), isPath, nr);
    end

end
