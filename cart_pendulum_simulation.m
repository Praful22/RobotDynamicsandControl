function cart_pendulum_simulation()
    % Define system parameters
    M = 1; % Mass of the cart (kg)
    m = 0.2; % Mass of the pendulum bob (kg)
    L = 0.3; % Length of the pendulum (m)
    g = 9.81; % Gravity (m/s^2)

    % Initial conditions
    x0 = 0; % Initial position of the cart (m)
    theta0 = pi + pi/6; % Initial angle of the pendulum (rad)
    dx0 = 0; % Initial velocity of the cart (m/s)
    dtheta0 = 0; % Initial angular velocity of the pendulum (rad/s)

    % State vector
    q0 = [x0; theta0];
    dq0 = [dx0; dtheta0];

    % Time span for simulation
    tspan = [0, 2]; % Simulate for 2 seconds

    % Solve for system dynamics using ode45
    [t, q] = ode45(@(t, q) cart_pendulum_dynamics(t, q, M, m, L, g), tspan, [q0; dq0]);

     % Extract states
    x = q(:, 1);
    theta = q(:, 2);

    % Plot x(t) and theta(t)
    figure;
    subplot(2, 1, 1);
    plot(t, x);
    xlabel('Time (s)');
    ylabel('x (m)');
    title('Cart Position over Time');
    grid on;

    subplot(2, 1, 2);
    plot(t, (theta-pi));
    xlabel('Time (s)');
    ylabel('\theta (rad)');
    title('Pendulum Angle over Time');
    grid on;

    % Animate the motion
    animate_cart_pendulum(t, q(:, 1), q(:, 2), L);
end

function dqdt = cart_pendulum_dynamics(t, q, M, m, L, g)
    % Extract states
    x = q(1);
    theta = q(2);
    dx = q(3);
    dtheta = q(4);

    % Compute system dynamics: These are values taken from 2b.
    D = [(M + m), m*L*cos(theta);
         m*L*cos(theta), m*L^2];

    C = [-m*L*dtheta^2*sin(theta); 0];

    G = [0; m*g*L*sin(theta)];

    tau = [0; 0]; % No control input for now

    % Solve for accelerations
    ddq = D \ (tau - C - G);

    % Construct dqdt
    dqdt = [dx; dtheta; ddq];
end

function animate_cart_pendulum(t, x, theta, L)
    % Animation parameters
    fps = 30; % Frames per second
    dt = 1 / fps;

    % Create figure
    figure;
    axis([-10*L, 10*L, -10*L, 10*L]);
    xlabel('x');
    ylabel('y');
    title('Cart-Pendulum Animation');

    % Plot cart-pendulum
    cart_width = 0.2;
    cart_height = 0.1;
    pendulum_radius = 0.02;

    cart = rectangle('Position', [x(1) - cart_width/2, -cart_height/2, cart_width, cart_height], 'Curvature', [0.1, 0.1], 'FaceColor', 'b');
    hold on;
    pendulum = plot([x(1), x(1) + L*sin(theta(1))], [0, -L*cos(theta(1))], 'r', 'LineWidth', 2);
    hold off;
    axis equal;
    grid on;
    pause(1);

    % Video creation
    v = VideoWriter("cart_pendulum_simulation.mp4", 'MPEG-4');
    v.FrameRate = 30; % Set frame rate
    open(v);

    % Animate motion
    for i = 1:length(t)
        % Update cart and pendulum position
        set(cart, 'Position', [x(i) - cart_width/2, -cart_height/2, cart_width, cart_height]);
        set(pendulum, 'XData', [x(i), x(i) + L*sin(theta(i))], 'YData', [0, -L*cos(theta(i))]);

        % Refresh plot
        drawnow;

        % Pause to achieve desired frame rate
        pause(dt);
        % Write frame to video
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
    close(v)
end


