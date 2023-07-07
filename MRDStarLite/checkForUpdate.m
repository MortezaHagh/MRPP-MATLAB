function [Open, Robot] = checkForUpdate(Open, GV, Robot, nodesForUpdate, Model)

    % update vertex
    [Open, Robot] = updateVertex(Open, GV, Robot, nodesForUpdate, Model);

end
