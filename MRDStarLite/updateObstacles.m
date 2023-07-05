function [GV, Model, nodesToUpdate] = updateObstacles(nodesToUpdate, GV, Model, t)

if  t==2
    newObstNode = 3;
    GV(newObstNode).robot = -1;
    Model.Predecessors{newObstNode,2} = Model.Predecessors{newObstNode,2}+inf;
    for iP=Model.Predecessors{newObstNode,1}
        indInSuc = Model.Successors{iP,1} == newObstNode;
        Model.Successors{iP,2}(indInSuc) = Model.Successors{iP,2}(indInSuc)+inf;
    end
    
    nodesToUpdate{end} = Model.Predecessors{newObstNode,1};
    
end

end