function sortedList = calculatePriority(Robots, robotList, Model)
% sort robots based on their calDistance from target nodes

h = zeros(numel(robotList), 1);
j=1;
for i=robotList
    xyP = Model.Nodes.cord(:, Robots(i).path(end));
    h(j)= calDistance(xyP(1), xyP(2), Robots(i).xt, Robots(i).yt, Model.distType);
    j=j+1;
end

[~, ind] = sort(h);
sortedList=robotList(ind);

end