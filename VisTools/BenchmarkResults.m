% Justin Kottinger
% this script is used to visualize benchmark results

% Just enter all the information above the astrisk line
% then, just run the script 

% the readlog function will extract all useful info about the planners
% feel free to change this function to use that information as you please

clear all; close all; clc;

LogFile = "/Users/Kotti/Desktop/MAPS-RRT/benchmarking_results/results.log";

Planners = ["control_RRT", "control_RRTplus"];

Titles = ["RRT", "RRTplus"];

NumRuns = 200;

% ***********************************************************************

[Headers, data] = ReadLog(LogFile, Planners, NumRuns);

% my research is mainly concerned with computation time, and number of
% solutions

% extract that information

% fun fact: the column "status" shows whether a correct solution was found
% or not in the log files... 5 for fail, 6 for correct

CompTimes = zeros(NumRuns, length(Planners));
NumSolutions = [];

for p = 1:length(Headers)
    NumFail = 0;
    NumSuccess = 0;
    for h = 1:length(Headers{p})
        if Headers{p}(h) == "time REAL"
            CompTimes(:, p) = data{p}(:, h);
        elseif Headers{p}(h) == "status ENUM"
            for i = 1:NumRuns
                if data{p}(i, 11) == 5
                    NumFail = NumFail + 1;
                elseif data{p}(i, 11) == 6
                    NumSuccess = NumSuccess + 1;
                end
            end
            NumSolutions = [NumSolutions NumSuccess];
        end
    end
end

% now, just plot that data 

Labels = {length(Titles)};
for i=1:length(Titles)
    Labels{i} = Titles(i);
end

% box plot of the computation times
figure(1);
hold on;
boxplot(CompTimes, 'labels', {Labels})
xlabel('Planners', 'FontSize',14);
ylabel('Time (s)', 'FontSize',14);
title('Comutation Times for Benchmark', 'FontSize',16);


% bar plot for NumSolutions
X = categorical(Titles);
figure(2);
bar(X, NumSolutions);
title('Number of Solutions Found', 'FontSize',16)




















