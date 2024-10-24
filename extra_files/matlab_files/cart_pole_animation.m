function cart_pole_animation
    % Parameters
    M = 1;    % mass of the cart
    m = 0.1;  % mass of the pendulum
    l = 0.5;  % length of the pendulum
    g = 9.81; % gravity

    % Initial conditions: [x, theta, dx, dtheta]
    s0 = [0; pi/4; 0; 0];  % Cart at 0, pendulum at 45 degrees

    % Time span for simulation
    tspan = [0 10];  % Simulate for 10 seconds

    % Solve ODE using ode45
    [t, s] = ode45(@(t, s) cart_pole_ode(t, s, M, m, l, g), tspan, s0);

    % Animation
    animate_cart_pole(t, s, l);
end

% ODE function defining the cart-pole system dynamics
function ds = cart_pole_ode(~, s, M, m, l, g)
    % State variables
    x = s(1);        % cart position
    theta = s(2);    % pendulum angle
    dx = s(3);       % cart velocity
    dtheta = s(4);   % pendulum angular velocity

    % Mass matrix M(q)
    Mq = [M + m, m*l*cos(theta); m*l*cos(theta), m*l^2];

    % Coriolis/Centrifugal matrix C(q, q_dot)
    Cq = [0, -m*l*sin(theta)*dtheta; 0, 0];

    % Gravity vector G(q)
    Gq = [0; -m*g*l*sin(theta)];

    % Control input (no control for open-loop simulation)
    tau = [0; 0];  % No external forces applied

    % Solve for accelerations (ddq = M(q)^-1 * (tau - C(q, q_dot)*q_dot - G(q)))
    ddq = Mq \ (tau - Cq * [dx; dtheta] - Gq);

    % Return state derivatives: [dx, dtheta, ddx, ddtheta]
    ds = [dx; dtheta; ddq];
end

% Function to animate the cart-pole system
function animate_cart_pole(t, s, l)
    % Create figure
    figure;
    cart_width = 0.3;  % Width of the cart
    cart_height = 0.2; % Height of the cart
    pole_length = l;   % Length of the pendulum

    for i = 1:length(t)
        % Clear the figure for each frame
        clf;

        % Get the current state
        x = s(i, 1);      % Cart position
        theta = s(i, 2);  % Pendulum angle

        % Calculate pendulum position
        pendulum_x = x + pole_length * sin(theta);
        pendulum_y = -pole_length * cos(theta);

        % Draw the cart
        rectangle('Position', [x - cart_width/2, -cart_height/2, cart_width, cart_height], 'Curvature', 0.1, 'FaceColor', [0 0.5 0.5]);

        % Draw the pendulum
        hold on;
        plot([x pendulum_x], [0 pendulum_y], 'k', 'LineWidth', 2);  % Pendulum rod
        plot(pendulum_x, pendulum_y, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');  % Pendulum bob

        % Set axis limits
        xlim([-2 2]);
        ylim([-1 1]);

        % Pause to create animation effect
        pause(0.005);
    end
end
