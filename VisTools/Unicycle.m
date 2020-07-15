function [t, State] = Unicycle(Xinit, Yinit, Vinit, Tinit, ControlInput, duration)
    
% Unicycle dynamics are as follows
% dx_dot = v * cos(theta)
% dy_dot = v * sin(theta)
% dv_dot = u1
% dtheta_dot = u2
% controls: c1 = sigma_dot and c2 = omega_dot

% initialize constants
    nSteps = 100;
    u1 = ControlInput(1, 1);
    u2 = ControlInput(1, 2);
    time = linspace(0, duration, nSteps);
%     set intial conditions
    State0 = [Xinit, Yinit, Vinit, Tinit];
%     integrate the state from initial state to final

    f = @(t, State) [State(3) * cos(State(4)), ...
        State(3) * sin(State(4)), u1, u2]';
%     
    [t, State] = ode45(f, time, State0);
 
end
