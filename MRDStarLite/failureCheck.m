function isFailed = failureCheck(Robots, robotList)

isFailed = false;
robotCount = numel(robotList);

if numel(Robots(1).path)<2
    return
end

stopCount = 0;
for iRobot= robotList
    if Robots(iRobot).path(end)==Robots(iRobot).path(end-1)
        stopCount = stopCount+1;
    end
end

if stopCount==robotCount
    isFailed = true;
end

end