function plotModelMulti(Model)

    xMin = Model.Map.xMin;
    xMax = Model.Map.xMax;
    yMin = Model.Map.yMin;
    yMax = Model.Map.yMax;
    obstX = Model.Obsts.x;
    obstY = Model.Obsts.y;
    Robot = Model.Robot;

    % figure(1)
    axis([xMin - 3, xMax + 3, yMin - 3, yMax + 3])
    axis equal
    % grid minor
    % grid on
    % box on
    hold on

    g = gca;
    g.XTick = [];
    g.YTick = [];
    set(g, 'XColor', 'none', 'YColor', 'none')

    Color = hsv(Model.robotCount);

    % start and target nodes
    for nr = 1:Model.robotCount
        % start
        plot(Robot(nr).xs, Robot(nr).ys, 's', 'MarkerSize', 10, 'MarkerEdgeColor', ...
            Color(nr, :), 'MarkerFaceColor', Color(nr, :));
        % target
        plot(Robot(nr).xt, Robot(nr).yt, 'diamond', 'MarkerSize', 10, 'MarkerEdgeColor', ...
            Color(nr, :), 'MarkerFaceColor', Color(nr, :));
        % text
        text(Robot(nr).xs, Robot(nr).ys + 0.2, num2str(nr), 'HorizontalAlignment', 'center', ...
            'FontName', 'Cambria', 'FontSize', 10) %,'FontWeight', 'bold' ys - 0.3
        text(Robot(nr).xt, Robot(nr).yt + 0.2, num2str(nr), 'HorizontalAlignment', 'center', ...
            'FontName', 'Cambria', 'FontSize', 10) % ,'FontWeight', 'bold' yt + 0.3
    end

    % Obstacles
    plot(obstX, obstY, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');

    % theta=linspace(0,2*pi,100);
    % for i=1:length(xc)
    %     fill(xc(i)+obst_r*cos(theta),yc(i)+obst_r*sin(theta),'k');
    % end

    % walls
    rectangle('Position', [xMin - 2, yMin - 2, (xMax - xMin + 3), (yMax - yMin + 3)], 'LineWidth', 2)

end
