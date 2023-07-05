function open_top_ind = selectTopNode_simple(open, targetNode, dir, nr)


open_top_ind = -1;  % 'no path!'

% no_visit: not_visited_id
no_visit = ~[open(nr).list.visited];

if sum(no_visit)>0
    candids = [[open(nr).list(no_visit).cost_f];
        abs([open(nr).list(no_visit).dir]-dir);
        find(no_visit)]';
    
    [~,ind_candids]=sortrows(candids(:,1:2));
    open_top_ind = candids(ind_candids(1), end);
end

end