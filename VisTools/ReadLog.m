function [ImportantProperties, data] = ReadLog(filename, planners, NumRuns)
% function reads a log file and returns two cells
% ImportantProperties contains the name of important info for each planner
% data returns the stored data for each Important property for each planner
% the cells are organized in the same order as given in "planners"

    log = fopen(filename);
    
    line1 = fgetl(log);
    
    done  = ["none"];
    
    ImportantProperties = {1, length(planners)};
    
    data = {1, length(planners)};
    
    while ischar(line1)
        foundplanner = 0;
        for i = 1:length(planners)
            
            if ismember(planners(i), done)
                continue
            else
%             name = "control_" + planners(i);
                plannerdata = strfind(line1, planners(i));
                if plannerdata == 1
                    % update flag
                    foundplanner = 1;
                    % do some stuff here
                    % find out how many common properties are in planner
                    line = fgetl(log);
                    common_props = str2double(line(1));
                    for j = 1:common_props
                        line_skip = fgetl(log);
                    end
                    % the next line tells us how many "important"
                    % properties were collected... we need this info.
                    line = fgetl(log);
                    important_prop = str2double(line(1:2));
                    % next, the log file will list what these properties
                    % are... we also want this info
                    IPs = {1, important_prop};
                    
                    for l=1:important_prop
                        line = fgetl(log);
                        ImportantProperties{i} = [];
             
                        IPs{l} = line;
                    end
                    ImportantProperties{i} = IPs;
                    line = fgetl(log);
                    % the next line is how mant times he planner was run
                    % do not need this information
                    
                    % the next series of lines is a table
                    % size(NumRuns, important_prop)
                    ID = {1, NumRuns};
                    stats = zeros(NumRuns, important_prop);
                    for n=1:NumRuns
                        line = fgetl(log);
                        ID{n} = line;
                        Str = sprintf('%s;', ID{n});
                        Num = sscanf(Str, '%g;', [important_prop, inf]).';
                        stats(n, :) = Num;
                    end
                    data{i} = stats;

                    % now, we have all information for this planner
                    % add it to done list
                    done = [done planners(i)];
                    %     move to next line
                    line1 = fgetl(log);
                end
            end
        end
        if foundplanner == 0
            %     move to next line
            line1 = fgetl(log);
        end
        
    end

end