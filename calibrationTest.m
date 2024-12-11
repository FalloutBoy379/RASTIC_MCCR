% === Manual Calibration Script for CSPR Robot ===
% This script allows manual control of motors to position the pendant,
% collects pendant positions via motion capture, and calibrates motor positions and rope lengths.

clear; close all; clc;

%% === 1. Initialization ===

% -------------------------------
% A. Initialize NatNet Client
% -------------------------------
% Ensure that the NatNet SDK is installed and the 'natnet' class is available.

% Initialize NatNet client
natnetclient = natnet();
natnetclient.HostIP = '127.0.0.1';          % Replace with your NatNet server IP if different
natnetclient.ConnectionType = 'Multicast';  % Adjust if necessary (e.g., 'Unicast')

% Connect to the motion capture system
disp('Connecting to NatNet server...');
natnetclient.connect();

% Verify connection
if ~natnetclient.isConnected
    error('Failed to connect to the NatNet server. Check IP and network settings.');
end
disp('Connected to NatNet server.');

% -------------------------------
% B. Define Motor IPs and Initial Guess
% -------------------------------
% Define Motor IPs (as provided)
motorIps = ["192.168.0.162"; "192.168.0.116"; "192.168.0.219"; "192.168.0.182"];

% Initial guess for motor positions (in millimeters)
initialAnchorPoints = [
    -4795, 3200, -2220.23;
    -4795, 3200, 2748.19;
     2616.11, 3200, 2779.82;
     3011.83, 3200, -2166.38
];

% Number of motors
numMotors = length(motorIps);

% -------------------------------
% C. Set Up Live Visualization
% -------------------------------
% Create a figure for live plotting
figure('Name', 'CSPR Robot Calibration', 'NumberTitle', 'off');
hold on;
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Live Pendant Position and Motor Positions');
axis equal;

% Plot initial anchor points (motor positions)
motorScatter = scatter3(initialAnchorPoints(:,1), initialAnchorPoints(:,2), initialAnchorPoints(:,3), ...
    100, 'ro', 'filled', 'DisplayName', 'Initial Motor Positions');
legendEntries = {'Pendant Position', 'Motor Positions'};
legendHandle = legend(legendEntries, 'Location', 'best');

% Initialize pendant position scatter
pendantScatter = scatter3(NaN, NaN, NaN, 100, 'b.', 'DisplayName', 'Pendant Position');

% Initialize plot limits (adjust based on your workspace)
xlim([-6000, 6000]);
ylim([0, 6000]);
zlim([-6000, 6000]);

% -------------------------------
% D. Define Data Collection Parameters
% -------------------------------
% Preallocate data structure
dataCollection = struct('motorCommands', {}, 'pendantPosition', {});

%% === 2. Interactive Control and Data Collection ===

% --- Function to Update Live Plot ---
% Note: This function will be defined later as a local function.

% --- Function to Collect Data Point ---
% Note: This function will be defined later as a local function.

% --- Interactive Data Collection ---
disp('=== Manual Data Collection Started ===');
disp('You can now manually control the motors to move the pendant to desired positions.');
disp('After positioning, press Enter to capture the current pendant position.');
disp('Type "done" when you have finished collecting data points.');

% Number of data points to collect
maxDataPoints = 20;
currentDataPoint = 0;

while currentDataPoint < maxDataPoints
    userInput = input('Press Enter to capture position or type "done" to finish: ', 's');
    if strcmpi(userInput, 'done')
        break;
    end

    % Capture data point
    dataPoint = collectDataPoint(motorIps, natnetclient, 1, pendantScatter, motorScatter, initialAnchorPoints);
    dataCollection(end+1) = dataPoint;
    currentDataPoint = currentDataPoint + 1;
    fprintf('Data Point %d Collected.\n', currentDataPoint);
end

disp('=== Data Collection Completed ===');

%% === 3. Disconnect NatNet Client ===

% Disconnect from NatNet server
natnetclient.disconnect();
disp('Disconnected from NatNet server.');

%% === 4. Calibration via Optimization ===

% --- Function to Calibrate Motor Positions and Rope Lengths ---
% Note: This function will be defined later as a local function.

% Perform calibration to estimate motor positions and rope lengths
disp('Starting calibration...');
[estimatedMotorPositions, estimatedRopeLengths] = calibrateMotorPositions(dataCollection, numMotors, initialAnchorPoints);

% Display the results
disp('=== Calibration Results ===');
for i = 1:numMotors
    fprintf('Motor %d:\n', i);
    fprintf('  Estimated Position: [%.2f, %.2f, %.2f] mm\n', ...
        estimatedMotorPositions(i,1), estimatedMotorPositions(i,2), estimatedMotorPositions(i,3));
    fprintf('  Estimated Rope Length: %.2f mm\n', estimatedRopeLengths(i));
end

%% === 5. Update Live Visualization with Estimated Motor Positions ===

% Plot estimated motor positions
set(motorScatter, 'XData', estimatedMotorPositions(:,1), ...
                  'YData', estimatedMotorPositions(:,2), ...
                  'ZData', estimatedMotorPositions(:,3));

% Update legend
legendHandle.String = {'Pendant Position', 'Estimated Motor Positions'};

%% === 6. Verification and Validation ===

% Calculate prediction errors for collected data
errors = [];
for k = 1:length(dataCollection)
    P = dataCollection(k).pendantPosition;
    if any(isnan(P))
        continue;
    end
    % Predicted rope lengths based on estimated motor positions
    predicted_lengths = vecnorm(estimatedMotorPositions - P, 2, 2);
    % Compare with estimated rope lengths
    error_k = abs(predicted_lengths - estimatedRopeLengths');
    errors = [errors; error_k];
end

% Calculate mean and standard deviation of errors
meanError = mean(errors, 1);
stdError = std(errors, 0, 1);

% Display error statistics
disp('=== Calibration Error Statistics ===');
for i = 1:numMotors
    fprintf('Motor %d: Mean Error = %.2f mm, Std Dev = %.2f mm\n', ...
        i, meanError(i), stdError(i));
end

%% === 7. Final Plot Enhancement ===

% Enhance the 3D plot with estimated motor positions
legend('Pendant Position', 'Estimated Motor Positions');
view(3); % 3D view
hold off;

%% === 8. Save Calibration Results ===

% Optionally, save calibration results to a file
% Uncomment the following lines to save the results

% save('CalibrationResults.mat', 'estimatedMotorPositions', 'estimatedRopeLengths', 'dataCollection');
% disp('Calibration results saved to "CalibrationResults.mat".');

%% === Local Functions ===

% -------------------------------
% Local Function Definitions
% -------------------------------

% --- Function to Set Motor Speed ---
function success = setMotorSpeed(motorIp, speed_f)
    % setMotorSpeed - Sets the speed of the motor at the specified IP address.
    %
    % Syntax: success = setMotorSpeed(motorIp, speed_f)
    %
    % Inputs:
    %   motorIp - String specifying the motor's IP address (e.g., '192.168.0.162')
    %   speed_f - Numeric value representing the desired speed (units depend on motor API)
    %
    % Outputs:
    %   success - Boolean indicating whether the speed was successfully set

    try
        speed = int32(speed_f);
        if(speed == 0)
            success = controlMotorDirection(motorIp, 'stop');
        else
            % Construct the URL based on the motor's API endpoint for setting speed
            url = sprintf('http://%s/setSpeed?speed=%d', motorIp, speed);

            % Send the HTTP GET request to set the speed
            response = webread(url);

            % Check if the response indicates success (modify based on actual API response)
            if contains(response, 'OK', 'IgnoreCase', true)
                success = true;
            else
                warning('Motor %s responded with error: %s', motorIp, response);
                success = false;
            end
        end
    catch ME
        % Handle any errors during the HTTP request
        warning('Failed to set speed for motor %s: %s', motorIp, ME.message);
        success = false;
    end
end

% --- Function to Control Motor Direction ---
function success = controlMotorDirection(motorIp, direction)
    % controlMotorDirection - Commands the motor to release, pull, or stop.
    %
    % Syntax: success = controlMotorDirection(motorIp, direction)
    %
    % Inputs:
    %   motorIp   - String specifying the motor's IP address (e.g., '192.168.0.162')
    %   direction - String indicating the direction ('release', 'pull', 'stop')
    %
    % Outputs:
    %   success - Boolean indicating whether the direction command was successfully sent

    % Validate the direction input
    validDirections = {'release', 'pull', 'stop'};
    if ~ismember(lower(direction), validDirections)
        error('Invalid direction. Use "release", "pull", or "stop".');
    end

    try
        % Construct the URL based on the motor's API endpoint for setting direction
        url = sprintf('http://%s/command?cmd=%s', motorIp, lower(direction));

        % Send the HTTP GET request to set the direction
        response = webread(url);

        % Check if the response indicates success (modify based on actual API response)
        if contains(response, 'OK', 'IgnoreCase', true)
            success = true;
        else
            warning('Motor %s responded with error: %s', motorIp, response);
            success = false;
        end
    catch ME
        % Handle any errors during the HTTP request
        warning('Failed to set direction for motor %s: %s', motorIp, ME.message);
        success = false;
    end
end

% --- Function to Get Pendant Position ---
function position = getBodyPosition(natnetclient)
    position = getRigidBodyPosition(natnetclient);
end

% --- Function to Get Rigid Body Position ---
function position = getRigidBodyPosition(natnetclient)
    % Get the model description
    model = natnetclient.getModelDescription;

    % Define the rigid body ID or name (modify based on your setup)
    rigidBodyName = 'Pendant'; % Replace with your rigid body's name
    rigidBodyID = find(strcmp({model.RigidBodies.Name}, rigidBodyName));

    if isempty(rigidBodyID)
        error('Rigid body "%s" not found in the model description.', rigidBodyName);
    end

    % Get the current frame data
    data = natnetclient.getFrame;

    if data.RigidBodies(rigidBodyID).Tracked
        % Extract position (convert from meters to millimeters)
        position = [
            data.RigidBodies(rigidBodyID).x * 1000, ... % X in mm
            data.RigidBodies(rigidBodyID).y * 1000, ... % Y in mm
            data.RigidBodies(rigidBodyID).z * 1000 ...  % Z in mm
        ];
    else
        error('Rigid body "%s" is not currently tracked.', rigidBodyName);
    end
end

% --- Function to Update Live Plot ---
function updatePlot(pendantScatter, motorScatter, pos, motorPos)
    % Update pendant position
    set(pendantScatter, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

    % Update motor positions
    set(motorScatter, 'XData', motorPos(:,1), 'YData', motorPos(:,2), 'ZData', motorPos(:,3));

    drawnow;
end

% --- Function to Collect Data Point ---
function dataPoint = collectDataPoint(motorIps, natnetclient, pauseDuration, pendantScatter, motorScatter, currentMotorPositions)
    % Collects a single data point by prompting the user to move motors,
    % then records the pendant position.

    disp('--- New Data Point Collection ---');
    disp('Please manually set the motor speeds/directions to position the pendant as desired.');
    disp('Press Enter when ready to capture the pendant position.');

    % Wait for user input
    pause;

    % Capture pendant position
    try
        pos = getBodyPosition(natnetclient);
        disp(['Captured Pendant Position: [', num2str(pos, '%.2f'), '] mm']);
        set(pendantScatter, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    catch ME
        warning('Failed to get pendant position: %s', ME.message);
        pos = [NaN, NaN, NaN];
    end

    % Store data
    dataPoint.motorCommands = []; % Since motors are manually controlled, we don't store commands
    dataPoint.pendantPosition = pos;

    % Pause for visualization
    pause(pauseDuration);
end

% --- Function to Calibrate Motor Positions and Rope Lengths ---
function [estimatedMotorPositions, estimatedRopeLengths] = calibrateMotorPositions(dataCollection, numMotors, initialGuess)
    % calibrateMotorPositions - Estimates motor positions and rope lengths using optimization.
    %
    % Inputs:
    %   dataCollection - Structure array with motor commands and pendant positions
    %   numMotors      - Number of motors
    %   initialGuess   - Initial guess for motor positions (Nx3 matrix)
    %
    % Outputs:
    %   estimatedMotorPositions - Estimated motor positions (Nx3 matrix)
    %   estimatedRopeLengths    - Estimated rope lengths (Nx1 vector)

    % Number of data points
    numDataPoints = length(dataCollection);

    % Objective function to minimize
    function residuals = objective(vars)
        % vars contains motor positions and rope lengths
        motorPositions = reshape(vars(1:3*numMotors), [3, numMotors])';
        ropeLengths = vars(3*numMotors+1:end);

        residuals = [];

        for k = 1:numDataPoints
            P = dataCollection(k).pendantPosition;
            if any(isnan(P))
                continue; % Skip invalid data points
            end
            for i = 1:numMotors
                M = motorPositions(i, :);
                L = ropeLengths(i);
                residual(end+1, 1) = norm(P - M) - L; %#ok<AGROW>
            end
        end
    end

    % Initial guess: concatenate motor positions and estimated rope lengths
    motorPositionsInit = initialGuess; % Nx3 matrix
    % Estimate initial rope lengths based on initial positions and a reference pendant position
    % Use the first valid pendant position
    referencePos = [];
    for k = 1:length(dataCollection)
        if ~any(isnan(dataCollection(k).pendantPosition))
            referencePos = dataCollection(k).pendantPosition;
            break;
        end
    end
    if isempty(referencePos)
        error('No valid pendant positions found for initial rope length estimation.');
    end
    ropeLengthsInit = vecnorm(motorPositionsInit - referencePos, 2, 2);
    initialVars = [motorPositionsInit(:); ropeLengthsInit(:)];

    % Set optimization options
    options = optimoptions('lsqnonlin', ...
        'Display', 'iter', ...
        'Algorithm', 'trust-region-reflective', ...
        'MaxFunctionEvaluations', 1e4, ...
        'MaxIterations', 1e3, ...
        'TolFun', 1e-6, ...
        'TolX', 1e-6);

    % Run the optimization
    fprintf('Starting calibration optimization...\n');
    [estimatedVars, resnorm, residual, exitflag] = lsqnonlin(@objective, initialVars, [], [], options);

    if exitflag <= 0
        warning('Optimization may not have converged. Exit flag: %d', exitflag);
    else
        disp('Optimization converged successfully.');
    end

    % Extract estimated motor positions and rope lengths
    estimatedMotorPositions = reshape(estimatedVars(1:3*numMotors), [3, numMotors])';
    estimatedRopeLengths = estimatedVars(3*numMotors+1:end);
end
