function PIDtest()
    % PIDtest - Performs PID control on a CSPR robot and plots PID components and motor velocities live.
    %
    % This function controls four motors to position a central pendant using PID control.
    % It logs the Proportional (P), Integral (I), and Derivative (D) components for each axis
    % and the motor velocities at each time step, and visualizes these values in real-time.
    % It also provides graphical buttons to retension specific motors and an Emergency Stop (E-Stop).
    
    % === Parameters ===
    Kp = [1, 1, 1];    % Proportional gains for [x, y, z]
    Ki = [0, 0, 0];    % Integral gains for [x, y, z]
    Kd = [0, 0, 0];    % Derivative gains for [x, y, z]
    dt = 0.05;          % Time step (seconds)
    
    % Initialize error variables
    last_error = [0, 0, 0];
    integral_error = [0, 0, 0];
    
    % Anchor points for the motors (in millimeters)
    anchorPoints = [
        -4795, 3200, -2220.23;
        -4795, 3200, 2748.19;
         2616.11, 3200, 2779.82;
         3011.83, 3200, -2166.38
    ];
    
    % Motor IP addresses
    motorIps = ["192.168.0.162"; "192.168.0.116"; "192.168.0.219"; "192.168.0.182"];
    
    % Initialize NatNet client
    natnetclient = initializeNatNet();
    
    % Get the initial position of the pendant
    initialBodyPosition = getBodyPosition(natnetclient);
    
    % Define the target position (in millimeters)
    targetPosition = [0, 1800, 0];
    fprintf('Target Position: [%.2f, %.2f, %.2f] mm\n', targetPosition);
    
    % Define a cleanup object to stop all motors on interruption
    cleanupObj = onCleanup(@() stopAllMotors(motorIps));
    
    fprintf("Use the buttons below the plots to retension motors or activate Emergency Stop.\n");
    
    %% === Data Logging Initialization ===
    % Preallocate variables for efficiency
    maxIterations = 1000; % Maximum number of iterations to prevent infinite loops
    P_log = zeros(maxIterations, 3);          % Proportional components
    I_log = zeros(maxIterations, 3);          % Integral components
    D_log = zeros(maxIterations, 3);          % Derivative components
    velocity_log = zeros(maxIterations, 3);   % PID output velocities
    motorVel_log = zeros(maxIterations, length(motorIps)); % Motor velocities
    time_log = zeros(maxIterations, 1);        % Time stamps
    
    % Initialize iteration counter
    iter = 0;
    
    %% === Live Plot Initialization ===
    % Create a new figure for live plotting
    hFig = figure('Name', 'PID Components and Motor Velocities', 'NumberTitle', 'off');
    
    % --- Subplot 1: PID Components ---
    subplot(2, 1, 1);
    hold on;
    grid on;
    xlabel('Time (s)');
    ylabel('PID Components');
    title('Proportional, Integral, and Derivative Components');
    axis tight;
    
    % Initialize animated lines for each PID component and axis
    % Proportional
    P_x_line = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'P X');
    P_y_line = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'P Y');
    P_z_line = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'P Z');
    
    % Integral
    I_x_line = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'I X');
    I_y_line = animatedline('Color', 'g', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'I Y');
    I_z_line = animatedline('Color', 'b', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'I Z');
    
    % Derivative
    D_x_line = animatedline('Color', 'r', 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'D X');
    D_y_line = animatedline('Color', 'g', 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'D Y');
    D_z_line = animatedline('Color', 'b', 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'D Z');
    
    legend('show');
    
    % --- Subplot 2: Motor Velocities ---
    subplot(2, 1, 2);
    hold on;
    grid on;
    xlabel('Time (s)');
    ylabel('Motor Velocities');
    title('Motor Velocities Over Time');
    axis tight;
    
    % Initialize animated lines for each motor
    motorColors = {'r', 'g', 'b', 'k'};
    motorLines = gobjects(length(motorIps), 1);
    for i = 1:length(motorIps)
        motorLines(i) = animatedline('Color', motorColors{i}, 'LineWidth', 1.5, 'DisplayName', sprintf('Motor %d', i));
    end
    legend('show');
    
    %% === Retensioning Initialization ===
    % Flags to indicate if a motor is being retensioned
    retensioning_flags = false(length(motorIps), 1);
    
    % Define maximum allowed pendant velocity before stopping retensioning
    max_pendant_velocity = 50; % mm/s
    
    % Initialize last pendant position for velocity calculation
    last_pendant_position = initialBodyPosition;
    
    %% === Add GUI Buttons ===
    % Positioning buttons below the plots
    btnWidth = 120;
    btnHeight = 40;
    spacing = 20;
    numButtons = length(motorIps) + 1; % Retension buttons + E-Stop
    figurePosition = get(hFig, 'Position');
    figWidth = figurePosition(3);
    figHeight = figurePosition(4);
    
    % Calculate starting X position to center buttons
    totalBtnWidth = numButtons * btnWidth + (numButtons - 1) * spacing;
    startX = (figWidth - totalBtnWidth) / 2;
    startY = 10; % Distance from the bottom
    
    % Create Retension Buttons
    for i = 1:length(motorIps)
        uicontrol('Style', 'pushbutton', ...
                  'String', sprintf('Retension Motor %d', i), ...
                  'Position', [startX + (i-1)*(btnWidth + spacing), startY, btnWidth, btnHeight], ...
                  'Callback', @(src, event) retensionCallback(i));
    end
    
    % Create E-Stop Button
    uicontrol('Style', 'pushbutton', ...
              'String', 'E-Stop', ...
              'Position', [startX + length(motorIps)*(btnWidth + spacing), startY, btnWidth, btnHeight], ...
              'Callback', @(src, event) eStopCallback());
    
    %% === Control Loop ===
    while true
        iter = iter + 1;
        if iter > maxIterations
            warning('Maximum iterations reached.');
            break;
        end
        
        % Get the current position of the pendant
        try
            currentPosition = getBodyPosition(natnetclient);
        catch ME
            warning('Failed to get pendant position: %s', ME.message);
            pause(dt);
            continue;
        end
        
        if any(isnan(currentPosition))
            warning('Current position unavailable.');
            pause(dt);
            continue;
        end
        
        % Calculate pendant velocity
        pendant_velocity = (currentPosition - last_pendant_position) / dt;
        last_pendant_position = currentPosition;
        
        % Compute PID output for each axis
        velocity = zeros(1, 3);
        for axis_r = 1:3
            [velocity(axis_r), last_error(axis_r), integral_error(axis_r)] = ...
                axisPID(targetPosition(axis_r), currentPosition(axis_r), last_error(axis_r), integral_error(axis_r), dt, Kp(axis_r), Ki(axis_r), Kd(axis_r));
        end
        
        % Log PID components and velocity
        P_log(iter, :) = Kp .* (targetPosition - currentPosition);
        I_log(iter, :) = Ki .* integral_error;
        D_log(iter, :) = Kd .* ((targetPosition - currentPosition) - last_error) / dt;
        velocity_log(iter, :) = velocity;
        
        % Invert velocity for control (assuming negative velocity moves towards target)
        velocity = -velocity;
        
        % Limit the maximum velocity for safety
        max_vel = 200;
        vel_norm = norm(velocity);
        if vel_norm > max_vel
            velocity = (velocity / vel_norm) * max_vel;
        end
        
        % === Retensioning Logic ===
        for i = 1:length(motorIps)
            if retensioning_flags(i)
                % Define retensioning speed
                retension_speed = 50; % Adjust as needed
                
                % Send 'pull' command to reel in the rope
                controlMotorDirection(motorIps(i), 'pull');
                setMotorSpeed(motorIps(i), retension_speed);
                
                % Check if pendant is moving significantly
                if norm(pendant_velocity) > max_pendant_velocity
                    fprintf("Pendant affected by Motor %d during retensioning. Stopping retension.\n", i);
                    % Stop retensioning
                    controlMotorDirection(motorIps(i), 'stop');
                    setMotorSpeed(motorIps(i), 0);
                    retensioning_flags(i) = false;
                end
            end
        end
        
        % === Kinematics Solver ===
        % Solve kinematics to compute motor velocities based on PID output
        motorVelocities = solveKinematics(anchorPoints, currentPosition, velocity);
        motorVel_log(iter, :) = motorVelocities;
        
        % Send commands to the motors
        for i = 1:length(motorIps)
            if retensioning_flags(i)
                % If retensioning, override PID control for this motor
                % Retensioning is already handled above
                continue;
            end
            
            if motorVelocities(i) > 0
                controlMotorDirection(motorIps(i), 'pull');
            elseif motorVelocities(i) < 0
                controlMotorDirection(motorIps(i), 'release');
            else
                controlMotorDirection(motorIps(i), 'stop');
            end
            setMotorSpeed(motorIps(i), abs(motorVelocities(i)));
        end
        
        % Log current time
        time_log(iter) = iter * dt;
        
        % === Live Plot Updating ===
        % Update PID Components Plot
        subplot(2, 1, 1);
        addpoints(P_x_line, time_log(iter), P_log(iter, 1));
        addpoints(P_y_line, time_log(iter), P_log(iter, 2));
        addpoints(P_z_line, time_log(iter), P_log(iter, 3));
        addpoints(I_x_line, time_log(iter), I_log(iter, 1));
        addpoints(I_y_line, time_log(iter), I_log(iter, 2));
        addpoints(I_z_line, time_log(iter), I_log(iter, 3));
        addpoints(D_x_line, time_log(iter), D_log(iter, 1));
        addpoints(D_y_line, time_log(iter), D_log(iter, 2));
        addpoints(D_z_line, time_log(iter), D_log(iter, 3));
        drawnow limitrate;
        
        % Update Motor Velocities Plot
        subplot(2, 1, 2);
        for i = 1:length(motorIps)
            addpoints(motorLines(i), time_log(iter), motorVel_log(iter, i));
        end
        drawnow limitrate;
        
        % Wait for the next iteration
        pause(dt);
    end
    
    %% === Plotting After Control Loop ===
    % (Optional) If you want to keep or enhance the final plots after the loop,
    % you can add additional plotting code here.
    
    %% === Helper Functions ===
    
    % --- Function to Initialize NatNet Client ---
    function natnetclient = initializeNatNet()
        % initializeNatNet - Initializes and connects the NatNet client.
        %
        % Outputs:
        %   natnetclient - Connected NatNet client object
    
        natnetclient = natnet;
        natnetclient.HostIP = '192.168.0.103';    % Adjust to OptiTrack server IP
        natnetclient.ClientIP = '192.168.0.185';  % Adjust to your local IP
        natnetclient.ConnectionType = 'Multicast';
        natnetclient.connect;
        if ~natnetclient.IsConnected
            error('Failed to connect to NatNet server.');
        end
        disp('NatNet client initialized and connected.');
    end
    
    % --- Function to Stop All Motors ---
    function stopAllMotors(motorIps)
        % stopAllMotors - Sends a stop command to all motors.
        %
        % Inputs:
        %   motorIps - Array of motor IP addresses
    
        for i = 1:length(motorIps)
            try
                controlMotorDirection(motorIps(i), 'stop');
                setMotorSpeed(motorIps(i), 0);
                fprintf("Stopped motor at IP: %s\n", motorIps(i));
            catch ME
                fprintf("Failed to stop motor at IP: %s. Error: %s\n", motorIps(i), ME.message);
            end
        end
    end
    
    % --- Function to Compute PID Output for One Axis ---
    function [vel_output, last_error_out, integral_error_out] = axisPID(target, current, last_error, integral_error, dt, Kp, Ki, Kd)
        % axisPID - Computes velocity for one axis based on position error using PID control.
        %
        % Inputs:
        %   target         - Target position for the axis
        %   current        - Current position of the axis
        %   last_error     - Error from the previous step
        %   integral_error - Cumulative error (integral term)
        %   dt             - Time step
        %   Kp, Ki, Kd     - PID gains
        %
        % Outputs:
        %   vel_output         - Velocity command for the axis
        %   last_error_out     - Updated error for the next step
        %   integral_error_out - Updated integral term for the next step
    
        % Compute error
        error = target - current;
    
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
        last_error_out = error;
        integral_error_out = integral_error;
    end
    
    %% === Nested Callback Functions ===
    
    % --- Callback Function for Retensioning a Motor ---
    function retensionCallback(motorIndex)
        % retensionCallback - Initiates retensioning for a specific motor.
        %
        % Inputs:
        %   motorIndex - Index of the motor to retension (1 to 4)
    
        if motorIndex >=1 && motorIndex <= length(motorIps)
            if ~retensioning_flags(motorIndex)
                retensioning_flags(motorIndex) = true;
                fprintf("Retensioning Motor %d...\n", motorIndex);
            else
                fprintf("Motor %d is already being retensioned.\n", motorIndex);
            end
        else
            fprintf("Invalid Motor Index: %d\n", motorIndex);
        end
    end
    
    % --- Callback Function for Emergency Stop ---
    function eStopCallback()
        % eStopCallback - Activates the Emergency Stop by stopping all motors.
    
        fprintf("E-Stop activated via button. Stopping all motors and terminating...\n");
        stopAllMotors(motorIps);
        % Close the figure to exit the control loop
        close(gcf);
    end
end
