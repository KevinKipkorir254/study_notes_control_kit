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

    % Solve ODE
    [t, s] = ode45(@(t,s) cart_pole_ode(t, s, M, m, l, g), tspan, s0);

    % Plot results
    figure;
    subplot(2,1,1);
    plot(t, s(:,1));  % Plot x (cart position)
    ylabel('Cart Position');
    subplot(2,1,2);
    plot(t, s(:,2));  % Plot theta (pendulum angle)
    ylabel('Pendulum Angle');

    %%figure;
    %%plot(s(:,1), s(:,3)) 

    figure;
    plot(s(:,2), s(:,4));
end

function ds = cart_pole_ode(~, s, M, m, l, g)
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

   %attempt 1 to add the control law
    % Define variables
    ke = 1;
    kv = 1;
    kx = 10^-1;
    kdelta = 0.01;

    %forming z
    z2 = dx;
    z5 = dtheta;
    z3 = cos(theta);

    zeds = [ z2; z5];

    E = (1/2)*zeds' * Mq * zeds + m*g*l*(z3 - 1);

    % Calculate the numerator
    numerator = kv * m * sin(theta) * (g * cos(theta) - l*dtheta^2) - (M + m * (sin(theta))^2) * (kx*x + kdelta*dx);

   % Calculate the denominator
    denominator = kv + (M + m * sin(theta)^2) * ke*E;

   %control input
    f = numerator/denominator;

    % Control input (no control for open-loop simulation)
    tau = [ f; 0];  % No external forces applied

    % Solve for accelerations
    ddq = Mq \ (tau - Cq * [dx; dtheta] - Gq);

    % State derivatives
    ds = [dx; dtheta; ddq];
end
