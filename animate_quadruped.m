function animate_quadruped(t, q, pF1, pF2, pF3, pF4)
    % Extract states
    x = q(:, 1);
    y = q(:, 2);
    z = q(:, 3);
    R_vec = q(:, 4:12);
    
    % Define body dimensions
    c = 0.3; % m
    b= 0.5; % m
    a = 0.15; % m
    
    vert1 = [b/2;c/2;a/2];
    vert2 = [b/2;-c/2;a/2];
    vert3 = [b/2;-c/2;-a/2];
    vert4 = [b/2;c/2;-a/2];
    vert5 = [-b/2;c/2;a/2];
    vert6 = [-b/2;-c/2;a/2];
    vert7 = [-b/2;-c/2;-a/2];
    vert8 = [-b/2;c/2;-a/2];

    % Define foot positions relative to inertial frame
    foot_positions = [pF1, pF2, pF3, pF4];
    
    % Create figure and axis
    figure;
    axis equal;
    axis([-0.5, 0.5, -0.5, 0.5, 0, 0.5]);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Quadruped Robot Animation');
    hold on;
    view(3);

    % Animation parameters
    fps = 30; % Frames per second
    dt = 1 / fps;

    % Video creation
    v = VideoWriter("Quadruped_simulation.mp4", 'MPEG-4');
    v.FrameRate = 30; % Set frame rate
    open(v);

    
    
    % Animate robot motion
    for i = 1:length(t)
        % Calculate rotation matrix
        R = reshape(R_vec(i,:),[3,3]);
        
        % Pause to control animation speed
        pause(0.1);
        
        % Clear previous frame
        clf;
        hold on;

        vert1world = transformPointToWorldFrame(vert1,R,[x(i);y(i);z(i)]);
        vert2world = transformPointToWorldFrame(vert2,R,[x(i);y(i);z(i)]);
        vert3world = transformPointToWorldFrame(vert3,R,[x(i);y(i);z(i)]);
        vert4world = transformPointToWorldFrame(vert4,R,[x(i);y(i);z(i)]);
        vert5world = transformPointToWorldFrame(vert5,R,[x(i);y(i);z(i)]);
        vert6world = transformPointToWorldFrame(vert6,R,[x(i);y(i);z(i)]);
        vert7world = transformPointToWorldFrame(vert7,R,[x(i);y(i);z(i)]);
        vert8world = transformPointToWorldFrame(vert8,R,[x(i);y(i);z(i)]);

        body_vertices = [vert1world,vert2world,vert3world,vert4world,...
            vert5world, vert6world, vert7world, vert8world];

        body_faces = [1 2 3 4;1 5 8 4;1 2 6 5;2 3 7 6;5 6 7 8;3 4 8 7];
        
        %Plot feet
        for i = 1:size(foot_positions, 2)
            foot_position = foot_positions(:, i);
            plot3(foot_position(1), foot_position(2), foot_position(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            quiver3(foot_position(1), foot_position(2), foot_position(3),0,0,0.25525,0,'LineWidth',2,'Color','b');
        end

        axis equal;
        axis([-0.5, 0.5, -0.5, 0.5, -0.5, 0.5]);
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('Quadruped Robot Animation');
        view(3);
        % Plot body
        patch('Vertices', body_vertices', 'Faces', body_faces, 'FaceColor', 'r', 'EdgeColor', 'k');
        pause(dt);
        % Write frame to video
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
    close(v);
end

function p_world = transformPointToWorldFrame(p_body, R, d)
    % Construct homogeneous transformation matrix
    T = eye(4);
    T(1:3, 1:3) = R; % Set rotation part of the matrix
    T(1:3, 4) = d;   % Set translation part of the matrix
    
    % Apply homogeneous transformation
    p_homogeneous = [p_body; 1]; % Convert to homogeneous coordinates
    p_world_homogeneous = T * p_homogeneous;
    
    % Convert back to Cartesian coordinates
    p_world = p_world_homogeneous(1:3);
end