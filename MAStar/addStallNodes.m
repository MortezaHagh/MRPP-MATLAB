function [ot_ind, np_count, open] = addStallNodes(np_count, open, targetNode, dir, nr, msc)

    ot_ind = -1;

    while np_count < msc && ot_ind == -1
        np_count = np_count + 1;
        disp(['robot: ', num2str(nr), ',  np_count: ', num2str(np_count)])

        target_10 = open(nr).List.nodeNumber;

        if any(target_10)
            disp('')
        end

        % delete visited nodes which were infeasible
        visit_ind = [open(nr).List.visited] == 1;
        visit_count = sum(visit_ind);
        open(nr).List = open(nr).List(find(visit_ind));
        open(nr).count = visit_count;

        tag_1_ind = [open(nr).List.tag] == 1;
        tag_1_count = sum(tag_1_ind);
        tag_1_ind = find(tag_1_ind);

        % sort open list based on x
        [~, inds] = sort([open(nr).List.fCost], 'ascend'); % descend ascend
        open(nr).List = open(nr).List(inds);

        % create stall nodes for each node in open
        temp_osize = open(nr).count;
        k = 1;

        if tag_1_count > 0

            for i = k:tag_1_count
                j = i + temp_osize;
                open(nr).List(j) = open(nr).List(tag_1_ind(i));
                open(nr).List(tag_1_ind(i)).tag = 0;
                open(nr).List(j).tag = 1;
                open(nr).List(tag_1_ind(i)).visited = 1;
                open(nr).List(j).visited = -1;
                open(nr).List(j).pNode = open(nr).List(j).nodeNumber;
                open(nr).List(j).gCost = open(nr).List(j).gCost + 1;
                open(nr).List(j).fCost = open(nr).List(j).fCost + 1; % +i*10
                open(nr).List(j).time = open(nr).List(j).time + 1;
                open(nr).count = open(nr).count + 1;
                ot_ind = selectTopNode_1(open, targetNode, dir, nr);

                if ot_ind ~= -1
                    break
                end

            end

            %         open(nr).count = open(nr).count+tag_1_count;
            %         ot_ind = selectTopNode_1(open, targetNode, dir, nr);
        end

    end

end
