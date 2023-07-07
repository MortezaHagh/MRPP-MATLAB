function [t, robo] = collisionsCheck2(paths, robotCount)

    t = 0;
    robo = 0;

    pl = zeros(1, robotCount);

    for j = 1:robotCount
        pl(j) = numel(paths(j).nodeNumbers);
    end

    [~, ind] = sort(pl, 'descend');

    % collision check
    for i = 1:robotCount - 1

        for j = i + 1:robotCount
            jl = pl(ind(j));
            il = pl(ind(i));

            for k = 1:il

                if k <= jl

                    if paths(ind(i)).nodeNumbers(k) == paths(ind(j)).nodeNumbers(k)
                        disp(['collision type1: robot ', num2str(ind(i)) ' and robot ' num2str(ind(j)) ', in k: ' num2str(k)])
                    end

                    if k < il && k < jl && (paths(ind(i)).nodeNumbers(k) == paths(ind(j)).nodeNumbers(k + 1)) ...
                            && (paths(ind(i)).nodeNumbers(k + 1) == paths(ind(j)).nodeNumbers(k))
                        disp(['collision type2: robot ', num2str(ind(i)), ' and robot ', num2str(ind(j)), ', in k: ', num2str(k)])
                    end

                end

                if k > jl

                    if paths(ind(i)).nodeNumbers(k) == paths(ind(j)).nodeNumbers(jl)
                        disp(['collision type3: robot ', num2str(ind(i)) ' and robot ' num2str(ind(j)) ', in k: ' num2str(k)])
                        robo = ind(j);
                        t = 3;
                        return
                    end

                end

            end

        end

    end

end
