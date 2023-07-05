function Model=createModel_mrdsl_1(Model)
% Create Complete Model for MRPP


%% Map Size
Map.lim = 28;
Map.xMin = 1;
Map.xMax = Map.lim;
Map.yMin = 1;
Map.yMax = Map.lim;

Map.nX=Map.xMax-Map.xMin+1;
Map.nY=Map.yMax-Map.yMin+1;

%% robots data
robotCount=4;
Model.robotCount = robotCount;

% dir: direction
% randsample([0 90 180 270], 1);
Robot(1).dir = 0;
Robot(2).dir = 0;
Robot(3).dir = 0;
Robot(4).dir = 0;

% start & goal
Robot(1).xs = 14;   Robot(1).ys = 11;
Robot(1).xt = 14;   Robot(1).yt = 17;

Robot(2).xs = 11;  Robot(2).ys = 14;
Robot(2).xt = 20;   Robot(2).yt = 14;

Robot(3).xs = 5;   Robot(3).ys = 7;
Robot(3).xt = 1;   Robot(3).yt = 12;

Robot(4).xs = 1;   Robot(4).ys = 1;
Robot(4).xt = 20;   Robot(4).yt = 20;


%  start & goal - node numbers
for iRobot=1:robotCount
    Robot(iRobot).startNode = (Robot(iRobot).ys-Map.yMin)*(Map.nX) + Robot(iRobot).xs-Map.xMin+1;
    Robot(iRobot).targetNode = (Robot(iRobot).yt-Map.yMin)*(Map.nX) + Robot(iRobot).xt-Map.xMin+1;
end

%% Obstacle

% radius
Obst.r = 0.25;

% Obstacle coordinates
xc1=[4 4 4 6 6 6 8 8 8 10 10 10 12 12 12];
yc1=[4 5 6 4 5 6 4 5 6 4 5 6 4 5 6];

xc2=[xc1 xc1 xc1 xc1];
yc2=[yc1 yc1+6 yc1+12 yc1+18];

xc3=xc2+12;
yc3=yc2;

Obst.x=[xc2 xc3];
Obst.y=[yc2 yc3];

Obst.count = length(Obst.x);

% obstacle node numbers
Obst.nodeNumber = zeros(1,Obst.count);
for iObst = 1:Obst.count
    Obst.nodeNumber(iObst) = (Obst.y(iObst)-Map.yMin)*(Map.nX) + Obst.x(iObst)-Map.xMin+1;
end

%% nodes & adj data
iNode = 0;
for iy = Map.yMin:Map.yMax
    for ix = Map.xMin:Map.xMax
        iNode = iNode + 1;
        Nodes.cord(1:2, iNode) = [ix, iy]';     % node coordinates
    end
end
Nodes.count = iNode;

%% edge costs, G, RHS, GV

switch Model.adjType
    case '4adj'
        ixy = [1 0; 0 1; 0 -1; -1 0];
        nAdj=4;
    case '8adj'
        ixy = [1 0; 0 1; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1];
        nAdj=8;
end


% euclidean manhattan
switch Model.distType
    case 'manhattan'
        edgeLength=2;
    case 'euclidean'
        edgeLength=sqrt(2);
end

Successors = cell(Nodes.count,2);
Predecessors = cell(Nodes.count,2);

gv.robot = 0;
GV = repmat(gv,Nodes.count,1);

for iNode=1:Nodes.count
    if ~any(iNode==Obst.nodeNumber)
        xNode = Nodes.cord(1,iNode);
        yNode = Nodes.cord(2,iNode);
        for iAdj=1:nAdj
            ix=ixy(iAdj,1);
            iy=ixy(iAdj,2);
            newX = xNode+ix;
            newY = yNode+iy;
            
            % check if the Node is within array bound
            if(newX>=Map.xMin && newX<=Map.xMax) && (newY>=Map.yMin && newY<=Map.yMax)
                newNodeNumber = iNode+ix+iy*(Map.nX);
                
                if ~any(newNodeNumber==Obst.nodeNumber)
                    Successors{iNode,1} = [Successors{iNode,1}, newNodeNumber];
                    Predecessors{newNodeNumber,1} =  [Predecessors{newNodeNumber,1}, iNode];
                    if ix~=0 && iy~=0
                        cost = edgeLength;
                    else
                        cost = 1;
                    end
                    Successors{iNode,2} = [Successors{iNode,2}, cost];
                    Predecessors{newNodeNumber,2} =  [Predecessors{newNodeNumber,1}, cost];
                end
            end
        end
    else
        GV(iNode).robot = -1;
    end
end

% G, RHS, km
G = inf(robotCount, Nodes.count);
G = mat2cell(G, ones(1,robotCount), Nodes.count);
RHS = inf(robotCount, Nodes.count);
RHS = mat2cell(RHS, ones(1,robotCount), Nodes.count);

%% save model
Model.Nodes = Nodes;
Model.Robot = Robot;
Model.Obst = Obst;
Model.Map = Map;

Model.Predecessors=Predecessors;
Model.Successors=Successors;
Model.RHS=RHS;
Model.GV =GV;
Model.G=G;

%% plot model
% plotModelMulti(model);

end
