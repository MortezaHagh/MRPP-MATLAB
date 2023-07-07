function plotAnimation_3(Sol, Model, Color)
    % animated points and paths, concurrent.

    robotCount = Model.robotCount;

    h1 = cell(robotCount, 1);
    h2 = cell(robotCount, 1);

    for nr = 1:robotCount
        h1{nr} = animatedline('Marker', 'o', 'MarkerFaceColor', Color(nr, :), 'MarkerEdgeColor', Color(nr, :));
        addpoints(h1{nr}, Sol(nr).x(1), Sol(nr).y(1));
        h2{nr} = animatedline('Color', Color(nr, :), 'Color', Color(nr, :), 'LineWidth', 1);
        addpoints(h2{nr}, Sol(nr).x(1:2), Sol(nr).y(1:2));
        drawnow
    end

    pause(1)

    i = 2;
    flag = true(robotCount, 1);

    while any(flag)

        for nr = 1:robotCount

            if i > numel(Sol(nr).x)
                flag(nr) = false;
            end

            if flag(nr)
                addpoints(h1{nr}, Sol(nr).x(i), Sol(nr).y(i));
                addpoints(h2{nr}, Sol(nr).x(i - 1:i), Sol(nr).y(i - 1:i));
            end

        end

        drawnow
        pause(0.4)
        i = i + 1;
    end

end
