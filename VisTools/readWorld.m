function [Model, Dim, NumVs, NumCtrls, Bndry, Obs, Start, Goal] = readWorld(FilePath)
world = fopen(FilePath);

% obstacles (needs to be initialized here to nothing)
Obs = zeros(6,1);


% 
model = 'DynModel';
dim = 'Dimension';
NumVehicles = 'NumVehicles';
NumControls = 'NumControls';
bndrys = 'boundary';
obstacles = 'obstacle';
strt = 'start';
gol = 'goal';

% this while loop reads the world file line by line and extracts
%   meaningful information
% it does this by searching for the keys
% once found, it extracts the floats from end of key to the end of line

% Start with first line of file
line1 = fgetl(world);
while ischar(line1)
%     initialize index of those keys
    modelI = strfind(line1, model);
    dimI = strfind(line1, dim);
    VehI = strfind(line1, NumVehicles);
    ContI = strfind(line1, NumControls);
    bndryI = strfind(line1, bndrys);
    ObsI = strfind(line1, obstacles);
    StartI = strfind(line1, strt);
    GolI = strfind(line1, gol);
%  if the index is 1, then we are at the line containing the key
%  if such event occurs, then we need to extract the relevant data
    if modelI == 1
        Model = sscanf(line1(modelI(1) + length(model):end), '%s');
    elseif dimI == 1
        Dim = sscanf(line1(dimI(1) + length(dim):end), '%g');
        
    elseif VehI == 1
        NumVs = sscanf(line1(VehI(1) + length(NumVehicles):end), '%g');
        
    elseif ContI == 1
        NumCtrls = sscanf(line1(ContI(1) + length(NumControls):end), '%g');
        
    elseif bndryI == 1
        Bndry = sscanf(line1(bndryI(1) + length(bndrys):end), '%g');
        
    elseif ObsI == 1
%         this is dynamically allocated because we do not know num of obs
%         aprioris
        Obs(end + 1: end + 6, 1) = sscanf(line1(ObsI(1) + ...
            length(obstacles):end), '%g');
        
    elseif StartI == 1
        Start = sscanf(line1(StartI(1) + length(strt):end), '%g');
        
    elseif GolI == 1
        Goal = sscanf(line1(GolI(1) + length(gol):end), '%g');
    end
    
%     move to next line

    line1 = fgetl(world);
end
end