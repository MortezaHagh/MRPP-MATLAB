function open = updateOpen(open, neighbors)

    % update or extend Open list with the successor nodes
    for i = 1:neighbors.count
        neighbors_open_common = [open.list.nodeNumber] == neighbors.list(i).nodeNumber & [open.list.visited] == 0;

        % if there is a common node in neighbors and open
        if any(neighbors_open_common)
            open_ind = find(neighbors_open_common);

            % if cost_f of node in exp_array is less than Open -> update node in Open
            if neighbors.list(i).cost_f < open.list(open_ind).cost_f
                open.list(open_ind) = neighbors.list(i);
            end

            % if there is No common node in exp_count and Open
        else
            % insert neighbor into the open list
            open.count = open.count + 1;
            open.list(open.count) = neighbors.list(i);
        end

    end

end
