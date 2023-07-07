function dist = calcalDistance(x1,y1,x2,y2, type)

if  strcmp(type,'euclidean')
    % euclidean calDistance
    dist = sqrt((x1-x2)^2 + (y1-y2)^2);
elseif strcmp(type,'manhattan')
    % manhattan calDistance
    dist = abs(x1-x2) + abs(y1-y2);
end

end