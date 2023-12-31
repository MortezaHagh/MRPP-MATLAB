function plotAnimation_2(Sol, Model, Color)
    % animated points with tale! concurrent.

    robotCount = Model.robotCount;

    h1 = zeros(robotCount, 1);
    h2 = zeros(robotCount, 1);

    for nr = 1:robotCount
        h1(nr) = plot(Sol(nr).x(1), Sol(nr).y(1), 'o', 'MarkerFaceColor', Color(nr, :), 'MarkerEdgeColor', Color(nr, :));
        h2(nr) = plot(Sol(nr).x(1:2), Sol(nr).y(1:2), 'Color', Color(nr, :), 'LineWidth', 2);
    end

    % create gif
    frame = getframe(gcf);
    img = frame2im(frame);
    [img, cmap] = rgb2ind(img, 256);
    imwrite(img, cmap, 'animation.gif', 'gif', 'LoopCount', Inf, 'DelayTime', 1);

    i = 2;
    pause(0.2)
    flag = true(robotCount, 1);

    while any(flag)

        for nr = 1:robotCount

            if i > numel(Sol(nr).x)
                flag(nr) = false;
            end

            if flag(nr)

                set(h1(nr), 'XData', Sol(nr).x(i))
                set(h1(nr), 'YData', Sol(nr).y(i))

                set(h2(nr), 'XData', Sol(nr).x(i - 1:i))
                set(h2(nr), 'YData', Sol(nr).y(i - 1:i))
            end

        end

        % draw
        drawnow
        pause(0.2)
        i = i + 1;

        % update gif
        frame = getframe(gcf);
        img = frame2im(frame);
        [img, cmap] = rgb2ind(img, 256);
        imwrite(img, cmap, 'animation.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
    end

end
