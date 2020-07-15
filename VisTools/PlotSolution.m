% Justin Kottinger
% ARIA Systems Research
% Created: 27 March 2020
clear; clc; close all;

% specify the problem

% dim = 3;
% nCtrls = 2;

% read in the world

WorldFilePath = "/Users/Kotti/Desktop/MAPS-RRT/txt/RSS_World_3Unicycle.txt";
[Model, Dim, NumVs, NumCtrls, Bndry, Obs, Start, Goal] = readWorld(WorldFilePath);
% read in the Solution
% 
SolFilePath = "/Users/Kotti/Desktop/MAPS-RRT/txt/path.txt";
[Xpos, Ypos, Vpos, THpos, Controls, Durations, Costs, path] = ...
    readPath(SolFilePath, Model, Dim, NumCtrls, NumVs);
% 
[nPoints, ~] = size(Xpos);

tolerance = 0.5;

if NumVs == 1
    figure(1);
    hold on;
%     in a kinematic car, this function plots 2D
%     xlim([Bndry(1), Bndry(4)])
%     ylim([Bndry(2), Bndry(5)])
    axis([Bndry(1), Bndry(4), Bndry(2), Bndry(5)])
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
                              Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)], ...
                          'FaceColor', "black")
    end
    scatter(Xpos(:), Ypos(:), 'b')
    title("1st Order Kinematic Car", 'FontSize', 14)
    legend(["Start", "Goal", "Path"], 'Location', 'Best', 'FontSize', 14)
    xlabel("x-position", 'FontSize', 14)
    ylabel("y-position", 'FontSize', 14)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
elseif NumVs == 2 && Model == "2KinematicCars"
    NumSegs = Costs(1);
    colors = ['r', 'b'];
    fig = 1;
    plotEntirePath(fig, colors, Model, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, Vpos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
    hold on;
    beginTime = 0;
    currTime = 0;
    for i = 1 : nPoints -1
        [nCols, ~] = size(Obs);
        nObs = nCols / 6;  % obstacle is defined by 6 points
        for j = 1 : nObs - 1  % minus one because we started with 6 zeros
            rectangle('Position',[Obs((6 * j) + 1, 1) Obs((6 * j) + 2, 1) ...
                              Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)], ...
                          'FaceColor', "black")
        end
        if Costs(i + 1) < NumSegs
            duration = [beginTime, currTime];
            title("Time Period = [" + sprintf('%.1f',duration(1)) + ", " ...
                + sprintf('%.1f',duration(2)) + "] seconds", 'FontSize', 14)
            filename = "solution/seg" + num2str(NumSegs);
            print(filename, '-dpng')
            NumSegs = NumSegs - 1;
            beginTime = currTime;
            fig = fig + 1;
            plotEntirePath(fig, colors, Model, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, Vpos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
        end
        for j = 1 : NumVs

            scatter(Start((Dim * (j - 1)) + 1), Start((Dim * (j - 1)) + 2), ...
                100, colors(j), 'filled', 'DisplayName', 'Start')
     
            [time, StateNew] = KinematicCar(Xpos(i, j), Ypos(i, j), ...
                THpos(i, j), ...
                Controls(i + 1, NumCtrls*(j - 1) + 1: NumCtrls*(j - 1) + 2), ...
                Durations(i + 1));
            p1 = plot(StateNew(:, 1), StateNew(:, 2), 'Color', colors(j), 'LineWidth', 2);
        end
        currTime = currTime + Durations(i + 1);
    end
    duration = [beginTime, currTime];
    title("Time Period = [" + sprintf('%.1f',duration(1)) + ", " ...
                + sprintf('%.1f',duration(2)) + "] seconds", 'FontSize', 14)
    xlabel("x-position", 'FontSize', 16)
    ylabel("y-position", 'FontSize', 16)
    axis equal;
    filename = "solution/seg" + num2str(NumSegs);
    print(filename, '-dpng')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif NumVs == 2 && Model == "2Linear"
    NumSegs = Costs(1);
    colors = ['r', 'b'];
    fig = 1;
    plotEntirePath(fig, colors, Model, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, Vpos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
    hold on;
    beginTime = 0;
    currTime = 0;
    for i = 1 : nPoints -1
        [nCols, ~] = size(Obs);
        nObs = nCols / 6;  % obstacle is defined by 6 points
        for j = 1 : nObs - 1  % minus one because we started with 6 zeros
            rectangle('Position',[Obs((6 * j) + 1, 1) Obs((6 * j) + 2, 1) ...
                              Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)], ...
                          'FaceColor', "black")
        end
        if Costs(i + 1) < NumSegs
            duration = [beginTime, currTime];
            title("Time Period = [" + sprintf('%.1f',duration(1)) + ", " ...
                + sprintf('%.1f',duration(2)) + "] seconds", 'FontSize', 14)
            NumSegs = NumSegs - 1;
            beginTime = currTime;
            fig = fig + 1;
            plotEntirePath(fig, colors, Model, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, Vpos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
        end
        for j = 1 : NumVs

            scatter(Start((Dim * (j - 1)) + 1), Start((Dim * (j - 1)) + 2), ...
                100, colors(j), 'filled', 'DisplayName', 'Start')
     
            [time, StateNew] = LinearCar(Xpos(i, j), Ypos(i, j), ...
                Controls(i + 1, NumCtrls*(j - 1) + 1: NumCtrls*(j - 1) + 2), ...
                Durations(i + 1));
            p1 = plot(StateNew(:, 1), StateNew(:, 2), 'Color', colors(j), 'LineWidth', 2);
        end
        currTime = currTime + Durations(i + 1);
    end
    duration = [beginTime, currTime];
    title("Time Period = [" + sprintf('%.1f',duration(1)) + ", " ...
                + sprintf('%.1f',duration(2)) + "] seconds", 'FontSize', 14)
    xlabel("x-position", 'FontSize', 16)
    ylabel("y-position", 'FontSize', 16)
%     axis equal;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif NumVs == 3 && Model == "3Linear"
    tolerance = 0.5;
    NumSegs = Costs(1);
    colors = {[1, 0, 0], [0, 0, 1], [0, 0.5, 0]};
    fig = 1;
    plotEntirePath(fig, colors, Model, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, Vpos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
    hold on;
    beginTime = 0;
    currTime = 0;
    for i = 1 : nPoints -1
        [nCols, ~] = size(Obs);
        nObs = nCols / 6;  % obstacle is defined by 6 points
        for j = 1 : nObs - 1  % minus one because we started with 6 zeros
            rectangle('Position',[Obs((6 * j) + 1, 1) Obs((6 * j) + 2, 1) ...
                              Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)], ...
                          'FaceColor', "black")
        end
        if Costs(i + 1) < NumSegs
            duration = [beginTime, currTime];
            title("Time Period = [" + sprintf('%.1f',duration(1)) + ", " ...
                + sprintf('%.1f',duration(2)) + "] seconds", 'FontSize', 14)
            NumSegs = NumSegs - 1;
            beginTime = currTime;
            fig = fig + 1;
            plotEntirePath(fig, colors, Model, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, Vpos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
        end
        for j = 1 : NumVs

            scatter(Start((Dim * (j - 1)) + 1), Start((Dim * (j - 1)) + 2), ...
                100, colors{j}, 'filled', 'DisplayName', 'Start')
     
            [time, StateNew] = LinearCar(Xpos(i, j), Ypos(i, j), ...
                Controls(i + 1, NumCtrls*(j - 1) + 1: NumCtrls*(j - 1) + 2), ...
                Durations(i + 1));
            p1 = plot(StateNew(:, 1), StateNew(:, 2), 'Color', colors{j}, 'LineWidth', 2);
        end
        currTime = currTime + Durations(i + 1);
    end
    duration = [beginTime, currTime];
    title("Time Period = [" + sprintf('%.1f',duration(1)) + ", " ...
                + sprintf('%.1f',duration(2)) + "] seconds", 'FontSize', 14)
    xlabel("x-position", 'FontSize', 16)
    ylabel("y-position", 'FontSize', 16)
    axis equal;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif NumVs == 3 && Model == "3Unicycle"
    tolerance = 1.0;
    NumSegs = Costs(1);
    colors = {[1, 0, 0], [0, 0, 1], [0, 0.5, 0]};
    fig = 1;
    plotEntirePath(fig, colors, Model, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, Vpos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
    hold on;
    beginTime = 0;
    currTime = 0;
    for i = 1 : nPoints -1
        [nCols, ~] = size(Obs);
        nObs = nCols / 6;  % obstacle is defined by 6 points
        for j = 1 : nObs - 1  % minus one because we started with 6 zeros
            rectangle('Position',[Obs((6 * j) + 1, 1) Obs((6 * j) + 2, 1) ...
                              Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)], ...
                          'FaceColor', "black")
        end
        if Costs(i + 1) < NumSegs
            duration = [beginTime, currTime];
            title("Time Period = [" + sprintf('%.1f',duration(1)) + ", " ...
                + sprintf('%.1f',duration(2)) + "] seconds", 'FontSize', 14)
            filename = "solution/seg" + num2str(fig);
            print(filename, '-dpng')
            NumSegs = NumSegs - 1;
            beginTime = currTime;
            fig = fig + 1;
            plotEntirePath(fig, colors, Model, Start, Goal, NumVs, Dim, tolerance, Xpos, Ypos, Vpos, THpos, Controls, NumCtrls, Durations, Obs, Costs, Bndry)
        end
        for j = 1:NumVs
            if (Controls(i + 1, NumCtrls*(j - 1) + 1) ~= 0)
                if (Controls(i + 1, NumCtrls*(j - 1) + 2) ~= 0)
                    scatter(Start((Dim * (j - 1)) + 1), Start((Dim * (j - 1)) + 2), ...
                    	100, colors{j}, 'filled', 'DisplayName', 'Start')
                    [~, StateNew] = Unicycle(Xpos(i, j), Ypos(i, j), Vpos(i, j), ...
                        THpos(i, j), Controls(i + 1, NumCtrls*(j - 1) + 1: NumCtrls*(j - 1) + 2), ...
                        Durations(i + 1));
                    p1 = plot(StateNew(:, 1), StateNew(:, 2), 'Color', colors{j}, 'LineWidth', 2);
                end
            end
        end
        currTime = currTime + Durations(i + 1);
    end
    duration = [beginTime, currTime];
    title("Time Period = [" + sprintf('%.1f',duration(1)) + ", " ...
                + sprintf('%.1f',duration(2)) + "] seconds", 'FontSize', 14)
    xlabel("x-position", 'FontSize', 16)
    ylabel("y-position", 'FontSize', 16)
%     axis equal;
    filename = "solution/seg" + num2str(fig);
    print(filename, '-dpng')
else
    disp("No implimentation of current requested set-up");
end






% video loop
% 
% colors = ['r', 'b'];
% GoalColors = {[1 0.75 0.75], [0.75 0.75 1]};
% NumSegs = Costs(1);
% figure(NumSegs);
% %     imshow(processo(:,:,1,i))
% PlotCircle(Goal, NumVs, Dim, tolerance, GoalColors)
% xlim([Bndry(1), Bndry(4)])
% ylim([Bndry(2), Bndry(5)])
% xlabel("x-position", 'FontSize', 16)
% ylabel("y-position", 'FontSize', 16)
% axis equal;
% box on;
% hold on;
% for i = 1 : nPoints - 1
%     [nCols, ~] = size(Obs);
%     nObs = nCols / 6;  % obstacle is defined by 6 points
%     for j = 1 : nObs - 1  % minus one because we started with 6 zeros
%         rectangle('Position',[Obs((6 * j) + 1, 1) Obs((6 * j) + 2, 1) ...
%             Obs((6 * j) + 4, 1) Obs((6 * j) + 5, 1)], ...
%             'FaceColor', "black")
%     end
%     states = {length(NumVs)};
%     for j = 1 : NumVs
%         scatter(Start((Dim * (j - 1)) + 1), Start((Dim * (j - 1)) + 2), ...
%                 100, colors(j), 'filled', 'DisplayName', 'Start')
%         [time, StateNew] = KinematicCar(Xpos(i, j), Ypos(i, j), ...
%                 THpos(i, j), ...
%                 Controls(i + 1, NumCtrls*(j - 1) + 1: NumCtrls*(j - 1) + 2), ...
%                 Durations(i + 1));
%         states{j} = StateNew;
%     end
%     % now we have both vehicles, need to plot them
%     % plot v1 first
%     for t = 1 : length(states{1}(:, 1))
%         hplot1 = draw_rectangle([states{1}(t, 1), states{1}(t, 2)], 0.2, 0.2, ...
%             states{1}(t, 3),colors(1));
%         hplot2 = draw_rectangle([states{2}(t, 1), states{2}(t, 2)], 0.2, 0.2, ...
%             states{2}(t, 3),colors(2));
%         F(i) = getframe(gcf);
%         drawnow;
%         delete(hplot1);
%         delete(hplot2);
%     end
% end
% % create the video writer with 10 fps
% writerObj = VideoWriter('myVideo.avi');
% writerObj.FrameRate = 1;
% % set the seconds per image
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(F)
%     % convert the image to a frame
%     frame = F(i) ;    
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);









