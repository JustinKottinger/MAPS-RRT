% Justin Kottinger
% this script is used to visualize benchmark results

% Just enter all the information above the astrisk line
% then, run the script 

% the readlog function will extract all useful info about the planners
% feel free to change this function to use that information as you please

clear all; close all; clc;

% LogFile = "/Users/Kotti/Desktop/MAPS-RRT/benchmarking_results/RSS_World/AllPlannersWithCosts.log";

LogFile = "/Users/Kotti/Desktop/MAPS-RRT/benchmarking_results/RSS_World_Hard/lim7/results_lim7_500.log";

% "control_MAPS-RRT", 
% Planners = ["control_MAPS-RRT", "control_MAPS-RRT Motion", "control_RRT", "control_RRTplus"];
% "control_MAPS-RRT Motion", 
Planners = ["control_MAPS-RRT Motion", "control_MAPS-RRT Cost"];

% Titles = ["MAPSRRT", "MAPSRRT Motion", "RRT", "RRTplus"];
%  
Titles = ["MAPSRRT Motion", "MAPSRRT Cost"];

NumRuns = 500;

MaxSolveTime = 100;
% ***********************************************************************

[Headers, data] = ReadLog(LogFile, Planners, NumRuns);

% my research is mainly concerned with computation time, costs, and number of
% solutions

% extract that information

% the column "status" shows whether a correct solution was found
% or not in the log files... 5 for fail, 6 for success

CompTimes = zeros(NumRuns, length(Planners));
CompTimes_success = cell(1, length(Planners));
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
                    CompTimes_success{p}(end + 1) = data{p}(i, 7);
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

Labels = cell(1, length(Titles));
for i=1:length(Titles)
    Labels{i} = Titles(i);
end

CompTimes_array = [];
max = 0;
for i=1:length(Planners)
   if length(CompTimes_success{i}) > max
      max = length(CompTimes_success{i});
      CompTimes_array = zeros(length(Planners), length(CompTimes_success{i})); 
   end
end

% turn cell into matrix for plot
for i=1:length(Planners)
   CompTimes_array(i, 1:length(CompTimes_success{i})) = CompTimes_success{i};
end

% box plot of the computation times
figure(1);
hold on;
boxplot((CompTimes + SegmentTimes), 'labels', {Labels})  %  Labels{1}
ylabel('Time (s)', 'FontSize',14);
title('Computation Time', 'FontSize',16);


% bar plot for NumSolutions
X = categorical(Titles);
figure(2);
bar(X, NumSolutions);
title('Number of Solutions Found', 'FontSize',16)
ylim([0, NumRuns])
 
figure(3);
hold on;
boxplot(Costs, 'labels', {Labels})  %  Labels{1}
% xlabel('Planners', 'FontSize',14);
ylabel('Number of Segments', 'FontSize',14);
title('Cost', 'FontSize',16);

% box plot of the computation times
figure(4);
hold on;
boxplot(CompTimes_array', 'labels', {Labels})  %   Labels{1}
ylabel('Time (s)', 'FontSize',14);
title('Computation Time Successful Trials', 'FontSize',16);


















