function openTopInd = selectTopNode_simple(Open, targetNode, dir, nr)

    openTopInd = -1; % 'no path!'

    % no_visit: not_visited_id
    no_visit = ~[Open(nr).List.visited];

    if sum(no_visit) > 0
        candids = [[Open(nr).List(no_visit).fCost];
                    abs([Open(nr).List(no_visit).dir] - dir);
                    find(no_visit)]';

        [~, ind_candids] = sortrows(candids(:, 1:2));
        openTopInd = candids(ind_candids(1), end);
    end

end
