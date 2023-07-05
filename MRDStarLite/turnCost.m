function dTheta = turnCost(currentNode, succNodes, Model, robotDir)

dTheta = zeros(1, numel(succNodes));
cnXY = Model.Nodes.cord(:, currentNode);
currentDir = deg2rad(robotDir);
i=1;
for iNode = succNodes
    nodeXY = Model.Nodes.cord(:, iNode);
    nodeTheta = atan2((nodeXY(2)-cnXY(2)), (nodeXY(1)-cnXY(1)));
    dTheta(i) = rad2deg(angdiff(currentDir, nodeTheta));
    i=i+1;
end

end