function plotModelMulti(Model)

    xmin = Model.Map.xMin;
    xmax = Model.Map.xMax;
    ymin = Model.Map.yMin;
    ymax = Model.Map.yMax;
    obstX = Model.Obst.x;
    obstY = Model.Obst.y;
    Robot = Model.Robot;

    % figure(1)
    axis([xmin - 3, xmax + 3, ymin - 3, ymax + 3])
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
    rectangle('Position', [xmin - 2, ymin - 2, (xmax - xmin + 3), (ymax - ymin + 3)], 'LineWidth', 2)

end
