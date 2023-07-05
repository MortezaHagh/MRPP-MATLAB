function [t, robo] = collisionsCheck2(paths, robot_count)

t=0;
robo = 0;

pl=zeros(1,robot_count);
for j=1:robot_count
    pl(j) = numel(paths(j).nodes);
end
[~,ind] = sort(pl, 'descend');

% collision check
for i=1:robot_count-1
    for j=i+1:robot_count
        jl = pl(ind(j));
        il = pl(ind(i));
        for k=1:il
            if k<=jl
                if  paths(ind(i)).nodes(k)==paths(ind(j)).nodes(k)
                    disp(['collision type1: robot ', num2str(ind(i)) ' and robot ' num2str(ind(j))  ', in k: ' num2str(k)])
                end
                if k<il && k<jl && (paths(ind(i)).nodes(k)==paths(ind(j)).nodes(k+1)) ...
                        && (paths(ind(i)).nodes(k+1)==paths(ind(j)).nodes(k))
                    disp(['collision type2: robot ', num2str(ind(i)), ' and robot ', num2str(ind(j)),  ', in k: ', num2str(k)])
                end
            end
            if k>jl
                if  paths(ind(i)).nodes(k)==paths(ind(j)).nodes(jl)
                    disp(['collision type3: robot ',num2str(ind(i)) ' and robot ' num2str(ind(j))  ', in k: ' num2str(k)])
                    robo=ind(j);
                    t=3;
                    return
                end
            end
        end
    end
end

end