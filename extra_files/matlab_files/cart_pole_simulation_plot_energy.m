function cart_pole_simulation
    % Parameters
    M = 1;  % mass of the cart
    m = 0.1;  % mass of the pendulum
    l = 0.5;  % length of the pendulum
    g = 9.81;  % gravity

    % Initial conditions
    s0 = [0; pi; 0; 0];  % [x, theta, dx, dtheta]

    % Time span
    tspan = [0 1000];

    % Solve ODE and store energy over time
    [t, s, energy] = ode45(@(t,s) cart_pole_ode(t, s, M, m, l, g), tspan, s0);

    % Plot Cart Position and Pendulum Angle
    figure;
    subplot(2,1,1);
    plot(t, s(:,1));  % Plot x (cart position)
    ylabel('Cart Position');
    subplot(2,1,2);
    plot(t, s(:,2));  % Plot theta (pendulum angle)
    ylabel('Pendulum Angle');
    
    % Plot Energy vs. Time
    figure;
    plot(t, energy);
    xlabel('Time (s)');
    ylabel('Energy (J)');
    title('Energy vs. Time');
end

function [ds, energy] = cart_pole_ode(~, s, M, m, l, g)
    x = s(1);
    theta = s(2);
    dx = s(3);
    dtheta = s(4);

    % Mass matrix M(q)
    Mq = [M + m, m*l*cos(theta); m*l*cos(theta), m*l^2];

    % Coriolis/Centrifugal forces
    Cq = [0, -m*l*sin(theta)*dtheta; 0, 0];

    % Gravity vector G(q)
    Gq = [0; -m*g*l*sin(theta)];

    % Energy calculation
    zeds = [dx; dtheta];
    E = (1/2)*zeds' * Mq * zeds + m*g*l*(cos(theta) - 1);

    % Swing-up control input
    ke = 1;
    kv = 1;
    kx = 0.1;
    kdelta = 0.01;

    numerator = kv * m * sin(theta) * (g * cos(theta) - l * dtheta^2) - (M + m * (sin(theta))^2) * (kx * x + kdelta * dx);
    denominator = kv + (M + m * sin(theta)^2) * ke * E;
    f_swingup = numerator / denominator;

    % Control input (no LQR for simplicity here)
    f = f_swingup;
    
    % Control input vector
    tau = [f; 0];

    % Solve for accelerations
    ddq = Mq \ (tau - Cq * [dx; dtheta] - Gq);

    % State derivatives
    ds = [dx; dtheta; ddq];

    % Return energy for plotting
    energy = E;
end
