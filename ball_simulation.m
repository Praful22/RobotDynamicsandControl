function ball_simulation()
    % Define initial conditions and parameters
    v0 = [1; 2]; % initial velocity vector in m/s
    m = 0.5; % mass in kg
    g = 9.81; % acceleration due to gravity in m/s^2
    c = 0.05; % drag coefficient
    tspan = [0 2]; % time span for simulation in seconds
    
    % Defining the function representing the differential equations
    f = @(t, v) [v(3); v(4); -c/m * norm(v(3:4)) * v(3); -g - c/m * norm(v(3:4)) * v(4)]; 
    
    % Solve the differential equations
    [t, v] = ode45(f, tspan, [0; 0;v0]);
    
    % Plot the trajectory
    plot(t, v(:,1));
    hold on
    plot(t, v(:,2));
    xlabel('time (s)');
    ylabel('Position (m)');
    title('Ball Trajectory');
    figure
    plot(t, v(:,3));
    hold on
    plot(t, v(:,4));
    xlabel('time (s)');
    ylabel('Velocity (m/s)');
end
