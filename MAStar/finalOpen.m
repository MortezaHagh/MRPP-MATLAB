function open = finalOpen(path)

    N = 10;
    path_len = numel(path.nodes);
    first.visited = 1;
    first.node = path.nodes(1);
    first.pnode = path.nodes(1);
    first.cost_g = 0;
    first.cost_h = 0;
    first.dir = 0;
    first.time = 0;
    open.list = first;

    for p = 2:path_len
        open.list(p) = first;
        open.list(p).node = path.nodes(p);
        open.list(p).pnode = path.nodes(p - 1);
        open.list(p).time = p - 1;
    end

    open.list(path_len + 1) = open.list(p);
    open.list(path_len + 1).pnode = open.list(path_len + 1).node;
    open.list(path_len + 1).time = open.list(path_len + 1).time + 1;

    for pp = path_len + 2:path_len + N
        open.list(pp) = open.list(pp - 1);
        open.list(pp).time = open.list(pp - 1).time + 1;
    end

    open.count = path_len + N;

end
