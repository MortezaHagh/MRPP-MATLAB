function Open = updateOpen(Open, neighbors)

    % update or extend Open list with the successor nodes
    for i = 1:neighbors.count
        neighbors_open_common = [Open.List.nodeNumber] == neighbors.List(i).nodeNumber & [Open.List.visited] == 0;

        % if there is a common node in neighbors and Open
        if any(neighbors_open_common)
            open_ind = find(neighbors_open_common);

            % if fCost of node in exp_array is less than Open -> update node in Open
            if neighbors.List(i).fCost < Open.List(open_ind).fCost
                Open.List(open_ind) = neighbors.List(i);
            end

            % if there is No common node in exp_count and Open
        else
            % insert neighbor into the Open list
            Open.count = Open.count + 1;
            Open.List(Open.count) = neighbors.List(i);
        end

    end

end
