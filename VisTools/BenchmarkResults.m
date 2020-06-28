% Justin Kottinger
% this script is used to visualize benchmark results

% Just enter all the information above the astrisk line
% then, just run the script 

% the readlog function will extract all useful info about the planners
% feel free to change this function to use that information as you please

clear all; close all; clc;

% LogFile = "/Users/Kotti/Desktop/MAPS-RRT/benchmarking_results/RSS_World/AllPlannersWithCosts.log";

LogFile = "/Users/Kotti/Desktop/MAPS-RRT/benchmarking_results/RSS_World/MAPS_Limit2Segments.log";

% "control_MAPS-RRT", 
% Planners = ["control_MAPS-RRT", "control_MAPS-RRT Motion", "control_RRT", "control_RRTplus"];

Planners = ["control_MAPS-RRT", "control_MAPS-RRT Motion"];

% Titles = ["MAPSRRT", "MAPSRRT Motion", "RRT", "RRTplus"];

Titles = ["MAPSRRT", "MAPSRRT Motion"];

NumRuns = 100;

MaxSolveTime = 60;
% ***********************************************************************

[Headers, data] = ReadLog(LogFile, Planners, NumRuns);

% my research is mainly concerned with computation time, costs, and number of
% solutions

% extract that information

% fun fact: the column "status" shows whether a correct solution was found
% or not in the log files... 5 for fail, 6 for correct

CompTimes = zeros(NumRuns, length(Planners));
SegmentTimes = zeros(NumRuns, length(Planners));
NumSolutions = [];
Costs = zeros(NumRuns, length(Planners));

for p = 1:length(Planners)
    NumFail = 0;
    NumSuccess = 0;
    for h = 1:length(Headers{p})
        if Headers{p}(h) == "time REAL"
            CompTimes(:, p) = data{p}(:, h);
        elseif Headers{p}(h) == "status ENUM"
            for i = 1:NumRuns
                if data{p}(i, h) == 5
                    NumFail = NumFail + 1;
                elseif data{p}(i, h) == 6
                    NumSuccess = NumSuccess + 1;
                end
            end
            NumSolutions = [NumSolutions NumSuccess];
        elseif Headers{p}(h) == "best cost REAL"
%             for i = 1:NumRuns
                Costs(:, p) = data{p}(:, h);
%             end
        elseif Headers{p}{h} == "segmenting time REAL"
            SegmentTimes(:, p) = data{p}(:, h);
        end
    end
end

% SegmentTimes
% Cost

% AvgCosts = [];
% for p = 1:length(Planners)
%     AvgCosts = [AvgCosts mean(Costs(p, :))];
% end

% 
% % now, just plot that data 
% 
Labels = {length(Titles)};
for i=1:length(Titles)
    Labels{i} = Titles(i);
end

% box plot of the computation times
figure(1);
hold on;
boxplot((CompTimes + SegmentTimes), 'labels', {Labels})
% xlabel('Planners', 'FontSize',14);
ylabel('Time (s)', 'FontSize',14);
title('Computation Time', 'FontSize',16);
% ylim([0, MaxSolveTime])


% bar plot for NumSolutions
X = categorical(Titles);
figure(2);
bar(X, NumSolutions);
title('Number of Solutions Found', 'FontSize',16)
ylim([0, NumRuns])

figure(3);
hold on;
% , 'labels', {Labels}
boxplot(Costs, 'labels', {Labels})
% xlabel('Planners', 'FontSize',14);
ylabel('Number of Segments', 'FontSize',14);
title('Cost', 'FontSize',16);

















