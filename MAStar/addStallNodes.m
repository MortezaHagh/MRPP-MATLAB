function [ot_ind, np_count, open] = addStallNodes(np_count, open, targetNode, dir, nr, msc)

ot_ind=-1;
while np_count<msc && ot_ind==-1
    np_count = np_count + 1;
    disp(['robot: ', num2str(nr), ',  np_count: ', num2str(np_count)])
    
    target_10 = open(nr).list.node;
    if any(target_10)
        disp('')
    end
    
    % delete visited nodes which were infeasible
    visit_ind = [open(nr).list.visited]==1;
    visit_count = sum(visit_ind);
    open(nr).list=open(nr).list(find(visit_ind));
    open(nr).count = visit_count;
    
    tag_1_ind = [open(nr).list.tag]==1;
    tag_1_count = sum(tag_1_ind);
    tag_1_ind = find(tag_1_ind);
    
    % sort open list based on x
    [~, inds] = sort([open(nr).list.cost_f], 'ascend'); % descend ascend
    open(nr).list = open(nr).list(inds);
    
    % create stall nodes for each node in open
    temp_osize = open(nr).count;
    k=1;
    if tag_1_count>0
        for i=k:tag_1_count
            j = i + temp_osize;
            open(nr).list(j) = open(nr).list(tag_1_ind(i));
            open(nr).list(tag_1_ind(i)).tag=0;
            open(nr).list(j).tag=1;
            open(nr).list(tag_1_ind(i)).visited = 1;
            open(nr).list(j).visited = -1;
            open(nr).list(j).pnode = open(nr).list(j).node;
            open(nr).list(j).cost_g = open(nr).list(j).cost_g + 1;
            open(nr).list(j).cost_f = open(nr).list(j).cost_f + 1;  % +i*10
            open(nr).list(j).time = open(nr).list(j).time+1;
            open(nr).count = open(nr).count+1;
            ot_ind = selectTopNode_1(open, targetNode, dir, nr);
            if ot_ind~=-1
                break
            end
        end
%         open(nr).count = open(nr).count+tag_1_count;
%         ot_ind = selectTopNode_1(open, targetNode, dir, nr);
    end
end

end