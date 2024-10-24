function cart_pole_simulation
    % Parameters
    M = 1;  % mass of the cart
    m = 0.1;  % mass of the pendulum
    l = 0.5;  % length of the pendulum
    g = 9.81;  % gravity
    energy_array = [];

    % Initial conditions
    s0 = [0; pi; 0; 0];  % [x, theta, dx, dtheta]

    % Time span
    tspan = [0 10];

    % Solve ODE
    [t, s] = ode45(@(t,s) cart_pole_ode(t, s, M, m, l, g, energy_array), tspan, s0);

    % Plot results
    figure;
    subplot(2,1,1);
    plot(t, s(:,1));  % Plot x (cart position)
    ylabel('Cart Position');
    subplot(2,1,2);
    plot(t, s(:,2));  % Plot theta (pendulum angle)
    ylabel('Pendulum Angle');

    figure;
    plot(s(:,2), s(:,4));  % Plot theta vs dtheta
    ylabel('Pendulum Angular Velocity');
    xlabel('Pendulum Angle');

    fopen(energy_array);
    
    % Plot Energy vs. Time
    figure;
    plot(energy_array);
    xlabel('Time (s)');
    ylabel('Energy (J)');
    title('Energy');
end

function ds = cart_pole_ode(~, s, M, m, l, g, energy_array)
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

    % Define variables for control
    ke = 1;
    kv = 1;
    kx = 0.1;
    kdelta = 0.01;

    % Energy
    z2 = dx;
    z5 = dtheta;
    z3 = cos(theta);
    zeds = [z2; z5];
    E = (1/2)*zeds' * Mq * zeds + m*g*l*(z3 - 1);
    energy_array(end+1) = E;
    

    % Control law for swing-up
    numerator = kv * m * sin(theta) * (g * cos(theta) - l * dtheta^2) - (M + m * (sin(theta))^2) * (kx * x + kdelta * dx);
    denominator = kv + (M + m * sin(theta)^2) * ke * E;
    f_swingup = numerator / denominator;
    fprintf("%.4f\r\n", E);

    % Check if the system is near the upright position (homoclinic orbit)
    % We consider a threshold for the total energy, when E is small, we switch to LQR
    energy_threshold = 0.0001;  % Tune this value for better performance
    if abs(E) < energy_threshold
        % Linearized system for LQR (about theta = 0)
        A = [0 0 1 0; 0 0 0 1; 0 -m*g/M 0 0; 0 (M + m)*g/(M*l) 0 0];
        B = [0; 0; 1/M; -1/(M*l)];
        Q = diag([10, 10, 1, 1]);  % State cost
        R = 1;  % Control cost

        % LQR gain
        K = lqr(A, B, Q, R);

        % State vector for LQR
        state = [x; theta; dx; dtheta];

        % Control input using LQR
        f_lqr = -K * state;

        % Apply LQR control
        f = f_lqr*2;
        %fprintf("n\r\n");
    else
        % Apply swing-up control
        f = f_swingup*10;
        %fprintf("h\r\n");
    end

    % Control input (torque or force applied to the cart)
    tau = [f; 0];

    % Solve for accelerations
    ddq = Mq \ (tau - Cq * [dx; dtheta] - Gq);

    % State derivatives
    ds = [dx; dtheta; ddq];
end
