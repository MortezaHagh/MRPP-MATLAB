function paths = MRPP_simple(Model)
    % MAstar, with time in open list
    % with stall node

    % Initialization and Parameters
    [closed, open, topnodes, robo, paths] = initialization_mh(Model);

    robot_count = Model.robot_count;
    ot_ind = zeros(robot_count, 1); % open_top_ind

    %% Start Algorithm

    % defining flags
    isPath = ones(robot_count, 1); % was pp successful?
    MissionFlag = true(robot_count, 1); % is pp for robot ended?

    while any(MissionFlag)

        for nr = 1:robot_count

            % update mission flag
            MissionFlag(nr) = (topnodes(nr).nodeNumber ~= robo(nr).targetNode && isPath(nr) == 1);

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
                ot_ind(nr) = selectTopNode_simple(open, robo(nr).targetNode, topnodes(nr).dir, nr);

                % update open & close with new topNode
                if ot_ind(nr) ~= -1
                    open(nr).list(ot_ind(nr)).visited = 1;
                    topnodes(nr) = open(nr).list(ot_ind(nr));
                    closed(nr).count = closed(nr).count + 1;
                    closed(nr).nodeNumbers(end + 1) = topnodes(nr).nodeNumber;
                else
                    isPath(nr) = 0;
                    disp(['No Path for robot ' num2str(nr) '!'])
                end

            end

        end

    end

    %% Optimal Path
    for nr = 1:robot_count
        paths(nr) = optimalPath_simple(Model, open(nr), isPath, nr);
    end

end
