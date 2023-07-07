function topNode = topKey(Open)

    keys = [[Open.List.key]', rand(Open.count, 1)];

    % search for node with min cost
    [~, sortInds] = sortrows(keys);
    topKeyInd = sortInds(1);

    topNode = Open.List(topKeyInd);
    topNode.ind = topKeyInd;

    if keys(topKeyInd, 1) == inf
        disp('topKey: No Path!')
    end

end
