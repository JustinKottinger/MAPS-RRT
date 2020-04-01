function [t, State] = KinematicCar(Xinit, Yinit, Tinit, ControlInput, duration)
    
% Kinematic Car dynamics are as follows
% dx = v*cos(theta)
% dy = v*sin(theta)
% dtheta = v*tan(phi)/length
% controls: c1 = v and c2 = phi
% length is constant: Currently = 0.2 
% If length is changed, make sure C++ code and here are consistant!

% initialize constants
    nSteps = 100;
    vel = ControlInput(1, 1);
    phi = ControlInput(1, 2);
    l = 0.2;
    time = linspace(0, duration, nSteps);
%     set intial conditions
    State0 = [Xinit, Yinit, Tinit];
%     integrate the state from initial state to final

    f = @(t, State) [vel * cos(State(3)), ...
        vel * sin(State(3)), (vel * tan(phi))/l]';
%     
    [t, State] = ode45(f, time, State0);
 
end