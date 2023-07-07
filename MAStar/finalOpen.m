function Open = finalOpen(path)

    N = 10;
    path_len = numel(path.nodeNumbers);
    first.visited = 1;
    first.nodeNumber = path.nodeNumbers(1);
    first.pNode = path.nodeNumbers(1);
    first.gCost = 0;
    first.hCost = 0;
    first.dir = 0;
    first.time = 0;
    Open.List = first;

    for p = 2:path_len
        Open.List(p) = first;
        Open.List(p).nodeNumber = path.nodeNumbers(p);
        Open.List(p).pNode = path.nodeNumbers(p - 1);
        Open.List(p).time = p - 1;
    end

    Open.List(path_len + 1) = Open.List(p);
    Open.List(path_len + 1).pNode = Open.List(path_len + 1).nodeNumber;
    Open.List(path_len + 1).time = Open.List(path_len + 1).time + 1;

    for pp = path_len + 2:path_len + N
        Open.List(pp) = Open.List(pp - 1);
        Open.List(pp).time = Open.List(pp - 1).time + 1;
    end

    Open.count = path_len + N;

end
