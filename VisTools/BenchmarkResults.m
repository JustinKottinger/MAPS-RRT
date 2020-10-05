% Justin Kottinger
% this script is used to visualize benchmark results

% Just enter all the information above the astrisk line
% then, run the script 

% the readlog function will extract all useful info about the planners
% feel free to change this function to use that information as you please

clear all; close all; clc;

% LogFile = "/Users/Kotti/Desktop/MAPS-RRT/benchmarking_results/RSS_World/AllPlannersWithCosts.log";

LogFile = "/Users/Kotti/Desktop/MAPS-RRT/benchmarking_results/Seperate_World/results_Seperate.log";

% "control_MAPS-RRT", 
% Planners = ["control_MAPS-RRT", "control_MAPS-RRT Motion", "control_RRT", "control_RRTplus"];
% "control_MAPS-RRT Motion", 
Planners = ["control_MAPS-RRT Motion", "control_MAPS-RRT Cost"];

% Titles = ["MAPSRRT", "MAPSRRT Motion", "RRT", "RRTplus"];
%  
Titles = ["MAPSRRT Motion", "MAPSRRT Cost"];

NumRuns = 1000;

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
Costs = cell(1, length(Planners));
States = zeros(NumRuns, length(Planners));
% Costs = zeros(NumRuns, length(Planners));
NumSolutions = [];

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
                    CompTimes_success{p}(end + 1) = data{p}(i, 8);
                end
            end
            NumSolutions = [NumSolutions NumSuccess];
        elseif Headers{p}(h) == "best cost REAL"
            for i = 1:NumRuns
                if data{p}(i, 7) == 6
                    Costs{p}(end + 1) = data{p}(i, h);
                end
            end
        elseif Headers{p}{h} == "segmenting time REAL"
            for i = 1:NumRuns
                if i == 1
                    SegmentTimes(i, p) = data{p}(i, h);
                else
                    SegmentTimes(i, p) = data{p}(i, h) - data{p}(i-1, h);
                end
                
            end
        elseif Headers{p}{h} == "graph states INTEGER"
            States(:, p) = data{p}(:, h);
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
   Costs_array(i, 1:length(Costs{i})) = Costs{i};
end


% box plot of the computation times
figure(1);
hold on;
boxplot((CompTimes), 'labels', {Labels})  %  Labels{1}
ylabel('Time (s)', 'FontSize',14);
title('Computation Time', 'FontSize',16);

figure(2);
for p=1:length(Planners)
    hold on;
    bar(p, NumSolutions(p))
    title('Number of Solutions Found ' + Labels{p}, 'FontSize',16)
    ylim([0, NumRuns])
end
set(gca, 'XTick', [1, 2])
set(gca,'xticklabel',Labels)
hold off;

% computation times
figure(3);
hold on;
boxplot(CompTimes_array', 'labels', {Labels})  %   Labels{1}
ylabel('Time (s)', 'FontSize',14);
title('Computation Time Successful Trials', 'FontSize',16);

% costs
figure(4);
boxplot(Costs_array', 'labels', {Labels})  %   Labels{1}
ylabel('Time (s)', 'FontSize',14);
title('Costs Successful Trials', 'FontSize',16); 

% segmentation time
figure(5);
boxplot(SegmentTimes, 'labels', {Labels})  %   Labels{1}
ylabel('Time (s)', 'FontSize',14);
title('Segment Times', 'FontSize',16); 

States_per_sec = zeros(NumRuns, length(Planners));
for p = 1:length(Planners)
    for i = 1:NumRuns
        States_per_sec(i, p) = States(i, p) / CompTimes(i, p);
    end
end

% number of total states
figure(6);
boxplot(States_per_sec, 'labels', {Labels})  %   Labels{1}
ylabel('States per Second', 'FontSize',14);
title('Number of States per Second', 'FontSize',16); 



















