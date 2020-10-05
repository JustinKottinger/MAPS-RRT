function [Xpos, Ypos, Vpos, THpos, Controls, Durations, Costs, path] = ...
    readPath(FilePath, model, dim, nControls, nVehicles)
  path = readmatrix(FilePath);
  [nSteps, ~] = size(path);
  Vpos = zeros(nSteps, nVehicles);
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
  elseif nVehicles == 2 && model == "2KinematicCars"
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
   elseif nVehicles == 2 && model == "2Linear"
       Xpos = zeros(nSteps, nVehicles);
       Ypos = zeros(nSteps, nVehicles);
       THpos = zeros(nSteps, nVehicles);
       Controls = zeros(nSteps, nControls * nVehicles);
       Durations = zeros(nSteps, 1);
       Costs = zeros(nSteps, 1);
       for i = 1 : nSteps
           Xpos(i, :) = [path(i, 1), path(i, dim + 1)];
           Ypos(i, :) = [path(i, 2), path(i, dim + 2)];
           Controls(i, :) = path(i, (dim*nVehicles) + 1: (dim*nVehicles) + (nVehicles*nControls));
           Durations(i) = path(i, end - 1);
           Costs(i) = path(i, end);
       end
   elseif nVehicles == 3 && model == "3Linear"
       Xpos = zeros(nSteps, nVehicles);
       Ypos = zeros(nSteps, nVehicles);
       THpos = zeros(nSteps, nVehicles);
       Controls = zeros(nSteps, nControls * nVehicles);
       Durations = zeros(nSteps, 1);
       Costs = zeros(nSteps, 1);
       for i = 1 : nSteps
           Xpos(i, :) = [path(i, 1), path(i, dim + 1), path(i, dim + 3)];
           Ypos(i, :) = [path(i, 2), path(i, dim + 2), path(i, dim + 4)];
           Controls(i, :) = path(i, (dim*nVehicles) + 1: (dim*nVehicles) + (nVehicles*nControls));
           Durations(i) = path(i, end - 1);
           Costs(i) = path(i, end);
       end
   elseif nVehicles == 3 && model == "3Unicycle"
       Xpos = zeros(nSteps, nVehicles);
       Ypos = zeros(nSteps, nVehicles);
       Vpos = zeros(nSteps, nVehicles);
       THpos = zeros(nSteps, nVehicles);
       Controls = zeros(nSteps, nControls * nVehicles);
       Durations = zeros(nSteps, 1);
       Costs = zeros(nSteps, 1);
       for i = 1 : nSteps
           Xpos(i, :) = [path(i, 1), path(i, dim + 1), path(i, dim + 5)];
           Ypos(i, :) = [path(i, 2), path(i, dim + 2), path(i, dim + 6)];
           Vpos(i, :) = [path(i, 3), path(i, dim + 3), path(i, dim + 7)];
           THpos(i, :) = [path(i, 4), path(i, dim + 4), path(i, dim + 8)];
           Controls(i, :) = path(i, (dim*nVehicles) + 1: (dim*nVehicles) + (nVehicles*nControls));
           Durations(i) = path(i, end - 1);
           Costs(i) = path(i, end);
       end
       
   elseif nVehicles == 2 && model == "2Unicycle"
       Xpos = zeros(nSteps, nVehicles);
       Ypos = zeros(nSteps, nVehicles);
       Vpos = zeros(nSteps, nVehicles);
       THpos = zeros(nSteps, nVehicles);
       Controls = zeros(nSteps, nControls * nVehicles);
       Durations = zeros(nSteps, 1);
       Costs = zeros(nSteps, 1);
       for i = 1 : nSteps
           Xpos(i, :) = [path(i, 1), path(i, dim + 1)];
           Ypos(i, :) = [path(i, 2), path(i, dim + 2)];
           Vpos(i, :) = [path(i, 3), path(i, dim + 3)];
           THpos(i, :) = [path(i, 4), path(i, dim + 4)];
           Controls(i, :) = path(i, (dim*nVehicles) + 1: (dim*nVehicles) + (nVehicles*nControls));
           Durations(i) = path(i, end - 1);
           Costs(i) = path(i, end);
       end
   elseif nVehicles == 3 && model == "3KinematicCars"
       Xpos = zeros(nSteps, nVehicles);
       Ypos = zeros(nSteps, nVehicles);
       Vpos = zeros(nSteps, nVehicles);
       THpos = zeros(nSteps, nVehicles);
       Controls = zeros(nSteps, nControls * nVehicles);
       Durations = zeros(nSteps, 1);
       Costs = zeros(nSteps, 1);
       for i = 1 : nSteps
           Xpos(i, :) = [path(i, 1), path(i, dim + 1), path(i, dim + 4)];
           Ypos(i, :) = [path(i, 2), path(i, dim + 2), path(i, dim + 5)];
           THpos(i, :) = [path(i, 3), path(i, dim + 3), path(i, dim + 6)];
           Controls(i, :) = path(i, (dim*nVehicles) + 1: (dim*nVehicles) + (nVehicles*nControls));
           Durations(i) = path(i, end - 1);
           Costs(i) = path(i, end);
       end
   end
   
end