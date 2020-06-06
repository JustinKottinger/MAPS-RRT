function [Xpos, Ypos, THpos, Controls, Durations, Costs, path] = ...
    readPath(FilePath, dim, nControls, nVehicles)
  path = readmatrix(FilePath);
  [nSteps, ~] = size(path);
  if nVehicles == 1
     Xpos = zeros(nSteps, nVehicles);
      Ypos = zeros(nSteps, nVehicles);
      THpos = zeros(nSteps, nVehicles);
      Controls = zeros(nSteps, nControls * nVehicles);
      Durations = zeros(nSteps, 1);
      for i = 1 : nSteps
          Xpos(i, :) = [path(i, 1)];
          Ypos(i, :) = [path(i, 2)];
          THpos(i, :) = [path(i, 3)];
          Controls(i, :) = path(i, 4:5);
          Durations(i) = path(i, end);
      end 
  elseif nVehicles == 2 % kinematic car
       Xpos = zeros(nSteps, nVehicles);
       Ypos = zeros(nSteps, nVehicles);
       THpos = zeros(nSteps, nVehicles);
       Controls = zeros(nSteps, nControls * nVehicles);
       Durations = zeros(nSteps, 1);
       Costs = zeros(nSteps, 1);
       for i = 1 : nSteps
           Xpos(i, :) = [path(i, 1), path(i, dim + 1)];
           Ypos(i, :) = [path(i, 2), path(i, dim + 2)];
           THpos(i, :) = [path(i, 3), path(i, dim + 3)];
           Controls(i, :) = path(i, (dim*nVehicles) + 1: (dim*nVehicles) + (nVehicles*nControls));
           Durations(i) = path(i, end - 1);
           Costs(i) = path(i, end);
       end
   end
   
end