function open_top_ind = selectTopNode_1(open, targetNode, dir, nr)

    flag = 0;
    flag2 = 1;
    robot_count = numel(open);
    rv = 1:robot_count;
    open_top_ind = -1; % 'no path!'

    % no_visit: not_visited_id
    no_visit = [open(nr).List.visited] ~= 1;
    snv = sum(no_visit);

    if snv > 0
        candids = [[open(nr).List(no_visit).fCost];
                    abs(angleDiff([open(nr).List(no_visit).dir], dir));
                    rand(1, snv);
                    find(no_visit)]';

        [~, ind_candids] = sortrows(candids(:, 1:3)); % 1:2
        candids_count = numel(ind_candids);

        for k = 1:candids_count
            open_top_ind = candids(ind_candids(k), end);
            flag = 1;

            for nnr = rv(rv ~= nr)
                % check for collision type 2 (angle_diff + pi) % (2*pi) - pi
                coll2 = [open(nnr).List.visited] == 1 & [open(nnr).List.pNode] == open(nr).List(open_top_ind).nodeNumber ...
                    & [open(nnr).List.nodeNumber] == open(nr).List(open_top_ind).pNode ...
                    & [open(nnr).List.time] == open(nr).List(open_top_ind).time;

                if any(coll2)
                    flag = 0;
                    break
                end

                % check for collision type 1
                coll1 = [open(nnr).List.visited] == 1 & [open(nnr).List.nodeNumber] == open(nr).List(open_top_ind).nodeNumber ...
                    & [open(nnr).List.time] == open(nr).List(open_top_ind).time;

                if any(coll1)
                    flag = 0;
                    break
                end

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %             if open(nr).List(open_top_ind).nodeNumber==targetNode
                %                 tg_id=[open(nnr).List.nodeNumber]==targetNode & [open(nnr).List.visited]==1;
                %                 if any(tg_id)
                %                     tg_id = find(tg_id);
                %                     tg_id = tg_id(end);
                %                     dt = open(nnr).List(tg_id).time-open(nr).List(open_top_ind).time;
                %                     if dt>0
                %                         flag=0;
                %                         % flag2=0;
                %                         break;
                %                     end
                %                 end
                %             end

            end

            if flag2 == 0
                break
            end

            if flag == 1 % successful
                break
            end

        end

    end

    if flag == 0 % fail
        open_top_ind = -1;
    end

end

% tg_nr_id = [open(nr).List.nodeNumber]==targetNode;
% tg_nr_id = find(tg_nr_id);
% tg_nr_id = tg_nr_id(end);
