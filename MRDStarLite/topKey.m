function TopNode = topKey(Open)

keys = [[Open.List.key]', rand(Open.count,1)];

% search for node with min cost
[~,sortInds]=sortrows(keys);
topKeyInd = sortInds(1);

TopNode = Open.List(topKeyInd);
TopNode.ind = topKeyInd;

if keys(topKeyInd,1)==inf
    disp('topKey: No Path!')
end

end
