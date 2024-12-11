function PIDVelocityTest()
% Parameters
Kp = [1, 10, 10]; % Proportional gains for [x, y, z]
Ki = [0, 0, 0];       % Integral gains for [x, y, z]
Kd = [0, 0, 0];       % Derivative gains for [x, y, z]
dt = 0.05; % Time step (seconds)

% Feedback adjustment gains
feedbackGain = 10; % Gain for feedback velocity adjustment

% Initialize error variables
last_error = [0, 0, 0];
integral_error = [0, 0, 0];
lastPosition = [0, 0, 0]; % Store last pendant position for velocity calculation

anchorPoints = [
    -4795, 3200, -2220.23;
    -4795, 3200, 2748.19;
     2616.11, 3200, 2779.82;
     3011.83, 3200, -2166.38
];

motorIps = ["192.168.0.162"; "192.168.0.116"; "192.168.0.219"; "192.168.0.182"];

% Initialize NatNet client
natnetclient = initializeNatNet();

% Initialize Joystick
joy = vrjoystick(1); % Create joystick object (adjust index as needed)
fprintf("Joystick initialized. Use left stick for X/Z and triggers for Y.\n");

% Define a cleanup object to stop all motors on interruption
cleanupObj = onCleanup(@() stopAllMotors(motorIps));

fprintf("Press 'q' to activate Emergency Stop (E-Stop).\n");

try
    % Get initial position of the pendant
    lastPosition = getBodyPosition(natnetclient);

    while true
        
        % Get the current position of the pendant
        currentPosition = getBodyPosition(natnetclient);
        if any(isnan(currentPosition))
            warning('Current position unavailable.');
            continue;
        end

        % Compute actual pendant velocity from position changes
        actualVelocity = (currentPosition - lastPosition) / dt;

        % Get joystick input for target velocity
        targetVelocity = getJoystickInput(joy);

        % Compute PID output for each axis
        velocity = zeros(1, 3);
        for axis_r = 1:3
            [velocity(axis_r), last_error(axis_r), integral_error(axis_r)] = ...
                axisPID(targetVelocity(axis_r), actualVelocity(axis_r), last_error(axis_r), integral_error(axis_r), dt, Kp(axis_r), Ki(axis_r), Kd(axis_r));
        end

        velocity = -velocity; % Negate velocity

        % Limit the maximum velocity for safety
        max_vel = 200;
        if norm(velocity) > max_vel
            velocity = (velocity / norm(velocity)) * max_vel
        end

        % Solve kinematics to compute motor velocities
        motorVelocities = solveKinematics(anchorPoints, currentPosition, velocity);

        % Adjust motor velocities based on feedback
        feedbackError = targetVelocity - actualVelocity;
        motorVelocities = motorVelocities + feedbackGain * solveKinematics(anchorPoints, currentPosition, feedbackError);

        % Send commands to the motors
        for i = 1:length(motorIps)
            if motorVelocities(i) > 0
                controlMotorDirection(motorIps(i), 'pull');
            elseif motorVelocities(i) < 0
                controlMotorDirection(motorIps(i), 'release');
            else
                controlMotorDirection(motorIps(i), 'stop');
            end
            setMotorSpeed(motorIps(i), abs(motorVelocities(i)));
        end

        % Update last position
        lastPosition = currentPosition;

        % Wait for the next iteration
        % pause(dt);
    end
catch ME
    % Handle errors gracefully and stop all motors
    fprintf("Error occurred: %s\n", ME.message);
    stopAllMotors(motorIps);
end

% Helper Functions
function natnetclient = initializeNatNet()
    % Initialize the NatNet client
    natnetclient = natnet;
    natnetclient.HostIP = '192.168.0.103'; % Adjust to OptiTrack server IP
    natnetclient.ClientIP = '192.168.0.185'; % Adjust to your local IP
    natnetclient.ConnectionType = 'Multicast';
    natnetclient.connect;
    if ~natnetclient.IsConnected
        error('Failed to connect to NatNet server.');
    end
end

function stopAllMotors(motorIps)
    % Stop all motors
    for i = 1:length(motorIps)
        try
            controlMotorDirection(motorIps(i), 'stop');
            fprintf("Stopped motor at IP: %s\n", motorIps(i));
        catch ME
            fprintf("Failed to stop motor at IP: %s. Error: %s\n", motorIps(i), ME.message);
        end
    end
end

% Get joystick input
function targetVelocity = getJoystickInput(joy)
    % Read joystick axes and buttons
    axes = axis(joy); % Get joystick axes values
    buttons = button(joy); % Get joystick button values

    % Map joystick inputs to target velocity
    targetVelocity = zeros(1, 3);
    targetVelocity(1) = 200 * axes(1); % X-axis velocity from left stick X
    targetVelocity(3) = 200 * axes(2); % Z-axis velocity from left stick Y
    targetVelocity(2) = 200 * (buttons(8) - buttons(7)); % Y-axis from triggers
end

function [vel_output, last_error, integral_error] = axisPID(target, actual, last_error, integral_error, dt, Kp, Ki, Kd)
    % axisPID - Computes velocity for one axis based on position error
    % Compute error
    error = target - actual;

    % Proportional term
    P = Kp * error;

    % Integral term
    integral_error = integral_error + error * dt;
    I = Ki * integral_error;

    % Derivative term
    D = Kd * (error - last_error) / dt;

    % Compute output velocity
    vel_output = P + I + D;

    % Update last_error for the next iteration
    last_error = error;
end
end
