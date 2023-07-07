function Paths = MRPP_1(Model)
    % with stall node & time

    % Initialization and Parameters
    [Closed, Open, topnodes, robo, Paths] = initializationMh(Model);

    robotCount = Model.robotCount;
    oTopInd = zeros(robotCount, 1); % openTopInd
    npCount = zeros(robotCount, 1); % no_path_count

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
                neighbors = expand(topnodes(nr), Closed(nr), Model, nr);

                % if strcmp(Model.adjType, '4adj')
                %     neighbors = neighbors4(topnodes(nr), Closed(nr), Model, nr);
                % elseif strcmp(Model.adjType, '8adj')
                %     neighbors = neighbors8(topnodes(nr), Closed(nr), Model, nr);
                % end

                % update or extend Open list with the successor nodes
                Open(nr) = updateOpen(Open(nr), neighbors);

                % select new Top Node
                oTopInd(nr) = selectTopNode_1(Open, robo(nr).targetNode, topnodes(nr).dir, nr);

                % if no path exists to the Target -> try stall nodes
                if oTopInd(nr) == -1 && npCount(nr) < Model.msc
                    [oTopInd(nr), npCount(nr), Open] = addStallNodes(npCount(nr), Open ...
                        , robo(nr).targetNode, topnodes(nr).dir, nr, Model.msc);
                end

                % update Open & close with new topNode
                if oTopInd(nr) ~= -1
                    Open(nr).List(oTopInd(nr)).visited = 1;
                    topnodes(nr) = Open(nr).List(oTopInd(nr));
                    Closed(nr).count = Closed(nr).count + 1;
                    Closed(nr).nodeNumbers(end + 1) = topnodes(nr).nodeNumber;
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
        Paths(nr) = optimalPath_1(Model, Open(nr), isPath, nr);
    end

end
