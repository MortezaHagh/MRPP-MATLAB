function open = finalOpen(path)

    N = 10;
    path_len = numel(path.nodeNumbers);
    first.visited = 1;
    first.nodeNumber = path.nodeNumbers(1);
    first.pNode = path.nodeNumbers(1);
    first.gCost = 0;
    first.hCost = 0;
    first.dir = 0;
    first.time = 0;
    open.List = first;

    for p = 2:path_len
        open.List(p) = first;
        open.List(p).nodeNumber = path.nodeNumbers(p);
        open.List(p).pNode = path.nodeNumbers(p - 1);
        open.List(p).time = p - 1;
    end

    open.List(path_len + 1) = open.List(p);
    open.List(path_len + 1).pNode = open.List(path_len + 1).nodeNumber;
    open.List(path_len + 1).time = open.List(path_len + 1).time + 1;

    for pp = path_len + 2:path_len + N
        open.List(pp) = open.List(pp - 1);
        open.List(pp).time = open.List(pp - 1).time + 1;
    end

    open.count = path_len + N;

end
