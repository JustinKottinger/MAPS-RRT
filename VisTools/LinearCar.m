function [t, State] = LinearCar(Xinit, Yinit, ControlInput, duration)
% Linear Car dynamics are as follows
% dx = vx
% dy = vy
% controls: c1 = vx and c2 = vy

% initialize constants
    nSteps = 100;
    vx = ControlInput(1, 1);
    vy = ControlInput(1, 2);
    time = linspace(0, duration, nSteps);
%     set intial conditions
    State0 = [Xinit, Yinit];
%     integrate the state from initial state to final

    f = @(t, State) [vx, vy]';
%     
    [t, State] = ode45(f, time, State0);
end

