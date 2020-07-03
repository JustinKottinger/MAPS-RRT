function plt = plotEntirePath(fig, colors, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
    [nPoints, ~] = size(Xpos);
    GoalColors = {[1 0.75 0.75], [0.75 0.75 1]};
    figure(fig);
    PlotCircle(Goal, NumVs, Dim, tolerance, GoalColors)
    xlim([Bndry(1), Bndry(4)])
    ylim([Bndry(2), Bndry(5)])
    xlabel("x-position", 'FontSize', 16)
    ylabel("y-position", 'FontSize', 16)
    axis equal;
    box on;
    for i = 1 : nPoints -1
        [nCols, ~] = size(Obs);
        nObs = nCols / 6;  % obstacle is defined by 6 points
        for j = 1 : nObs - 1  % minus one because we started with 6 zeros
            rectangle('Position',[Obs((6 * j) + 1, 1) Obs((6 * j) + 2, 1) ...
                              Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)], ...
                          'FaceColor', "black")
        end
        for j = 1 : NumVs
            scatter(Start((Dim * (j - 1)) + 1), Start((Dim * (j - 1)) + 2), ...
                100, colors(j), 'filled', 'DisplayName', 'Start')
            [~, StateNew] = KinematicCar(Xpos(i, j), Ypos(i, j), ...
                THpos(i, j), ...
                Controls(i + 1, NumCtrls*(j - 1) + 1: NumCtrls*(j - 1) + 2), ...
                Durations(i + 1));
            p1 = plot(StateNew(:, 1), StateNew(:, 2), 'Color', colors(j), 'LineWidth', 2);
            p1.Color(4) = 0.1;
        end
    end
    h = zeros(6, 1);
    h(1) = plot(NaN,NaN,'or', 'MarkerFaceColor',[1 0 0]);
    h(2) = plot(NaN, NaN, 'ob', 'MarkerFaceColor',[0 0 1]);
    h(3) = plot(NaN,NaN,'r');
    h(4) = plot(NaN,NaN,'b');
    h(5) = plot(NaN, NaN, '--r', 'MarkerFaceColor',[1 0 0]);
    h(6) = plot(NaN, NaN, '--b', 'MarkerFaceColor',[0 0 1]); 
    xlabel("x-position", 'FontSize', 16)
    ylabel("y-position", 'FontSize', 16)
    axis equal;








end

