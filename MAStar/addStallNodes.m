function [oTopInd, npCount, Open] = addStallNodes(npCount, Open, targetNode, dir, nr, msc)

    oTopInd = -1;

    while npCount < msc && oTopInd == -1
        npCount = npCount + 1;
        disp(['robot: ', num2str(nr), ',  npCount: ', num2str(npCount)])

        target_10 = Open(nr).List.nodeNumber;

        if any(target_10)
            disp('')
        end

        % delete visited nodes which were infeasible
        visit_ind = [Open(nr).List.visited] == 1;
        visit_count = sum(visit_ind);
        Open(nr).List = Open(nr).List(find(visit_ind));
        Open(nr).count = visit_count;

        tag_1_ind = [Open(nr).List.tag] == 1;
        tag_1_count = sum(tag_1_ind);
        tag_1_ind = find(tag_1_ind);

        % sort Open list based on x
        [~, inds] = sort([Open(nr).List.fCost], 'ascend'); % descend ascend
        Open(nr).List = Open(nr).List(inds);

        % create stall nodes for each node in Open
        temp_osize = Open(nr).count;
        k = 1;

        if tag_1_count > 0

            for i = k:tag_1_count
                j = i + temp_osize;
                Open(nr).List(j) = Open(nr).List(tag_1_ind(i));
                Open(nr).List(tag_1_ind(i)).tag = 0;
                Open(nr).List(j).tag = 1;
                Open(nr).List(tag_1_ind(i)).visited = 1;
                Open(nr).List(j).visited = -1;
                Open(nr).List(j).pNode = Open(nr).List(j).nodeNumber;
                Open(nr).List(j).gCost = Open(nr).List(j).gCost + 1;
                Open(nr).List(j).fCost = Open(nr).List(j).fCost + 1; % +i*10
                Open(nr).List(j).time = Open(nr).List(j).time + 1;
                Open(nr).count = Open(nr).count + 1;
                oTopInd = selectTopNode_1(Open, targetNode, dir, nr);

                if oTopInd ~= -1
                    break
                end

            end

            %         Open(nr).count = Open(nr).count+tag_1_count;
            %         oTopInd = selectTopNode_1(Open, targetNode, dir, nr);
        end

    end

end
