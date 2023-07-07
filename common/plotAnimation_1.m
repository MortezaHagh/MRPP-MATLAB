function plotAnimation_1(Sol, Model, Color)
    % animated points and Paths, in turn.

    robotCount = Model.robotCount;

    i = 1;
    flag = true(robotCount, 1);

    while any(flag)

        for nr = 1:robotCount

            if i > numel(Sol(nr).x)
                flag(nr) = false;
            end

            if flag(nr)
                plot(Sol(nr).x(i), Sol(nr).y(i), 'o', 'MarkerFaceColor', Color(nr, :), 'MarkerEdgeColor', Color(nr, :));
                text(Sol(nr).x(i) + 0.1, Sol(nr).y(i) + 0.1, num2str(i), 'Color', Color(nr, :))

                if i > 1
                    plot(Sol(nr).x(i - 1:i), Sol(nr).y(i - 1:i), 'Color', Color(nr, :), 'LineWidth', 2);
                end

                pause(0.2)
            end

        end

        i = i + 1;
    end

end
