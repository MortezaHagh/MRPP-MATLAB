function openTopInd = selectTopNode_1(Open, targetNode, dir, nr)

    flag = 0;
    flag2 = 1;
    robotCount = numel(Open);
    rv = 1:robotCount;
    openTopInd = -1; % 'no path!'

    % no_visit: not_visited_id
    no_visit = [Open(nr).List.visited] ~= 1;
    snv = sum(no_visit);

    if snv > 0
        candids = [[Open(nr).List(no_visit).fCost];
                    abs(angleDiff([Open(nr).List(no_visit).dir], dir));
                    rand(1, snv);
                    find(no_visit)]';

        [~, ind_candids] = sortrows(candids(:, 1:3)); % 1:2
        candids_count = numel(ind_candids);

        for k = 1:candids_count
            openTopInd = candids(ind_candids(k), end);
            flag = 1;

            for nnr = rv(rv ~= nr)
                % check for collision type 2 (angle_diff + pi) % (2*pi) - pi
                coll2 = [Open(nnr).List.visited] == 1 & [Open(nnr).List.pNode] == Open(nr).List(openTopInd).nodeNumber ...
                    & [Open(nnr).List.nodeNumber] == Open(nr).List(openTopInd).pNode ...
                    & [Open(nnr).List.time] == Open(nr).List(openTopInd).time;

                if any(coll2)
                    flag = 0;
                    break
                end

                % check for collision type 1
                coll1 = [Open(nnr).List.visited] == 1 & [Open(nnr).List.nodeNumber] == Open(nr).List(openTopInd).nodeNumber ...
                    & [Open(nnr).List.time] == Open(nr).List(openTopInd).time;

                if any(coll1)
                    flag = 0;
                    break
                end

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %             if Open(nr).List(openTopInd).nodeNumber==targetNode
                %                 tg_id=[Open(nnr).List.nodeNumber]==targetNode & [Open(nnr).List.visited]==1;
                %                 if any(tg_id)
                %                     tg_id = find(tg_id);
                %                     tg_id = tg_id(end);
                %                     dt = Open(nnr).List(tg_id).time-Open(nr).List(openTopInd).time;
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
        openTopInd = -1;
    end

end

% tg_nr_id = [Open(nr).List.nodeNumber]==targetNode;
% tg_nr_id = find(tg_nr_id);
% tg_nr_id = tg_nr_id(end);
