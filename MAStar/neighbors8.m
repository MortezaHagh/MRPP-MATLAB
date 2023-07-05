function neighbors = neighbors8(topnode, closed, model, nr)
% node pnode cost_g cost_f dir


xy = model.nodes.cord(:,topnode.node);
robot = model.robo(nr);
x = xy(1);
y = xy(2);
nc=0; % neighbors.count

dxdy = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
dirs = [0, 45, 90, 135, 180, 225, 270, 315];

for k= 1:8
    i=dxdy(k,1);
    j=dxdy(k,2);
    
    % dir
    nn_dir = dirs(k);
    
    % nn: new node
    if (i~=j || i~=0) % &&
        % The node itself is not its successor
        %(i==0 || j==0)  % eliminate corner nodes -> 4 node
        nn_x = x +i;
        nn_y = y +j;
        
        % check if the new node is within limits
        if((nn_x>=model.xmin && nn_x<=model.xmax) && (nn_y>=model.ymin && nn_y<=model.ymax))
            new_node = topnode.node+i+(j*(model.xmax-model.xmin+1));
            
            % check if it is in Closed list
            if ~any(new_node==closed.nodes)
                nc=nc+1;
                list(nc).visited = 0;
                list(nc).node = new_node;
                list(nc).pnode = topnode.node;
                list(nc).cost_g = topnode.cost_g + Distance(x, y, nn_x, nn_y, model.dist_type);
                cost_h = Distance(robot.xt, robot.yt, nn_x, nn_y, model.dist_type)*2;
                list(nc).cost_f = list(nc).cost_g + cost_h;
                list(nc).dir = nn_dir;
                list(nc).time = topnode.time+1;
                list(nc).tag=1;
            end
        end
    end
end

neighbors.count=nc;
if nc~=0
    neighbors.list = list;
else
    neighbors.list= [];
end

end