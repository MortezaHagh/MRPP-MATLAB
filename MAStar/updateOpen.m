function open = updateOpen(open, neighbors)

    % update or extend Open list with the successor nodes
    for i = 1:neighbors.count
        neighbors_open_common = [open.List.nodeNumber] == neighbors.List(i).nodeNumber & [open.List.visited] == 0;

        % if there is a common node in neighbors and open
        if any(neighbors_open_common)
            open_ind = find(neighbors_open_common);

            % if fCost of node in exp_array is less than Open -> update node in Open
            if neighbors.List(i).fCost < open.List(open_ind).fCost
                open.List(open_ind) = neighbors.List(i);
            end

            % if there is No common node in exp_count and Open
        else
            % insert neighbor into the open list
            open.count = open.count + 1;
            open.List(open.count) = neighbors.List(i);
        end

    end

end
