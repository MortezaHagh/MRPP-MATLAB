function collisionsCheck(Paths, robot_count)

pl=zeros(1,robot_count);
for j=1:robot_count
    pl(j) = numel(Paths(j).nodeNumbers);
end
[~,ind] = sort(pl, 'descend');

% collision check
for i=1:robot_count-1
    for j=i+1:robot_count
        jl = pl(ind(j));
        il = pl(ind(i));
        for k=1:il
            if k<=jl
                if  Paths(ind(i)).nodeNumbers(k)==Paths(ind(j)).nodeNumbers(k)
                    disp(['collision type1: robot ', num2str(ind(i)) ' and robot ' num2str(ind(j))  ', in k: ' num2str(k)])
                end
                if k<il && k<jl && (Paths(ind(i)).nodeNumbers(k)==Paths(ind(j)).nodeNumbers(k+1)) ...
                        && (Paths(ind(i)).nodeNumbers(k+1)==Paths(ind(j)).nodeNumbers(k))
                    disp(['collision type2: robot ', num2str(ind(i)), ' and robot ', num2str(ind(j)),  ', in k: ', num2str(k)])
                end
            end
            if k>jl
                if  Paths(ind(i)).nodeNumbers(k)==Paths(ind(j)).nodeNumbers(jl)
                    disp(['collision type3: robot ',num2str(ind(i)) ' and robot ' num2str(ind(j))  ', in k: ' num2str(k)])
                end
            end
        end
    end
end

end