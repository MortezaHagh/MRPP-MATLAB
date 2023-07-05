function plotSolution(Sol, Model, Color)

robotCount=Model.robotCount;
figure(1)
hold on
for nr=1:robotCount
    plot(Sol(nr).x, Sol(nr).y, 'color', Color(nr,:), 'LineWidth', 1);
    plot(Sol(nr).x(2:end-1), Sol(nr).y(2:end-1), 'o', 'MarkerFaceColor', Color(nr,:), 'MarkerEdgeColor', Color(nr,:), ...
        'MarkerSize', 3);
end

end