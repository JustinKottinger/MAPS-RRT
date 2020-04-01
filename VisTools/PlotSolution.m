% Justin Kottinger
% ARIA Systems Research
% Created: 27 March 2020
clear; close; clc;

% specify the problem

% dim = 3;
% nCtrls = 2;

% read in the world

WorldFilePath = "/Users/Kotti/Desktop/OMPLusingODESolvers/txt/world2agents.txt";
[Dim, NumVs, NumCtrls, Bndry, Obs, Start, Goal] = readWorld(WorldFilePath);

% read in the Solution
% 
SolFilePath = "/Users/Kotti/Desktop/OMPLusingODESolvers/txt/path.txt";
[Xpos, Ypos, THpos, Controls, Durations] = ...
    readPath(SolFilePath, Dim, NumCtrls, NumVs);
% 
[nPoints, ~] = size(Xpos);
% Plot the solution to the path
% circle = [goal(1) + 0.1*cos(linspace(0, 2 * pi)); ...
%     goal(2) + 0.1*sin(linspace(0, 2 * pi))];
if NumVs == 1
    figure(1);
    hold on;
%     in a kinematic car, this function plots 2D
    xlim([Bndry(1), Bndry(4)])
    ylim([Bndry(2), Bndry(5)])
    scatter(Start(1), Start(2), 100, 'r', 'filled')
    scatter(Goal(1), Goal(2), 100, 'g', 'filled')
%     current = [Xpos(1), Ypos(1), THpos(1)];
    for i = 1 : nPoints - 1
        [time, StateNew] = KinematicCar(Xpos(i), Ypos(i), ...
            THpos(i), Controls(i + 1, : ), Durations(i + 1));
        plot(StateNew(:, 1), StateNew(:, 2), 'b');
    end
% %     plot obstacles
    [nCols, ~] = size(Obs);
    nObs = nCols / 6;  % obstacle is defined by 6 points
    for j = 1 : nObs - 1  % minus one because we started with 6 zeros
        rectangle('Position',[Obs((6 * j) + 1, 1) Obs((6 * j) + 2, 1) ...
                              Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)])
    end
    scatter(Xpos(:), Ypos(:), 'b')
    title("1st Order Kinematic Car", 'FontSize', 14)
    legend(["Start", "Goal", "Path"], 'Location', 'Best', 'FontSize', 14)
    xlabel("x-position", 'FontSize', 14)
    ylabel("y-position", 'FontSize', 14)
elseif NumVs == 2 
    figure(1);
    hold on;
    xlim([Bndry(1), Bndry(4)])
    ylim([Bndry(2), Bndry(5)])
    colors = ['r', 'b'];
    for j = 1 : NumVs
        scatter(Goal((Dim * (j - 1)) + 1), Goal((Dim * (j - 1)) + 2), ...
            100, colors(j), 'filled', 'DisplayName', 'Goal')
        scatter(Start((Dim * (j - 1)) + 1), Start((Dim * (j - 1)) + 2), ...
            100, 'k', 'filled', 'DisplayName', 'Start')
        current = [Xpos(1, j), Ypos(1, j), THpos(1, j)];
        for i = 1 : nPoints - 1
%         [time, StateNew] = KinematicCar(current(1), current(2), ...
%             current(3), Controls(i + 1,:), Durations(i + 1));
%         current = StateNew(end, :);
        
            [time, StateNew] = KinematicCar(Xpos(i, j), Ypos(i, j), ...
                THpos(i, j), ...
                Controls(i + 1, NumCtrls*(j - 1) + 1: NumCtrls*(j - 1) + 2), ...
                Durations(i + 1));
            plot(StateNew(:, 1), StateNew(:, 2), 'Color', colors(j));
        end
    scatter(Xpos(:, j), Ypos(:, j), colors(j));
    end
    [nCols, ~] = size(Obs);
    nObs = nCols / 6;  % obstacle is defined by 6 points
    for j = 1 : nObs - 1  % minus one because we started with 6 zeros
        rectangle('Position',[Obs((6 * j) + 1, 1) Obs((6 * j) + 2, 1) ...
                              Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)])
    end
    h = zeros(5, 1);
    h(1) = plot(NaN,NaN,'r');
    h(2) = plot(NaN,NaN,'b');
    h(3) = plot(NaN,NaN,'ok', 'MarkerFaceColor',[1 1 1]);
    h(4) = plot(NaN, NaN, 'or', 'MarkerFaceColor',[1 0 0]);
    h(5) = plot(NaN, NaN, 'ob', 'MarkerFaceColor',[0 0 1]); 
    legend(h, 'Vehicle 1 Path','Vehicle 2 Path','Start', ...
        'Vehicle 1 Goal', 'Vehicle 2 Goal', 'FontSize', 12, 'Location', 'Best');
    title("Two Vehicles: 1st Order Kinematic Cars", 'FontSize', 12)
%     legend(["Start", "Goal", "Vehicle 1 Path", "Vehicle 2 Path"], 'Location', 'Best', 'FontSize', 14)
    xlabel("x-position", 'FontSize', 12)
    ylabel("y-position", 'FontSize', 12)
else
    disp("No implimentation of current requested set-up");
end