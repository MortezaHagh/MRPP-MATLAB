function [open, Robot] = checkForUpdate(open, GV, Robot, nodesForUpdate, Model)

    % update vertex
    [open, Robot] = updateVertex(open, GV, Robot, nodesForUpdate, Model);

end
