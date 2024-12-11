% MATLAB Script: Real-Time Motion Capture Visualization with NatNet
%
% This script visualizes the real-time position of a rigid body ('MCCR') using
% NatNet and periodically updates the rope lengths connecting anchor points to 'MCCR'.
%
% Ensure that the NatNet SDK is installed and the necessary MATLAB files are on the path.
% Include the NatNet client functions or add their directory to the MATLAB path.
% Example:
% addpath('C:\Path\To\NatNetSDK\Lib\MATLAB');

%% 1. Define Rotation Matrix and Rotate Coordinates

% Define rotation angle (90 degrees in radians)
theta = 90; 
theta_rad = deg2rad(theta);

% Define rotation matrix to rotate Y-axis to positive Z-axis (rotate +90 degrees about X-axis)
R = [1, 0, 0;
     0, cos(theta_rad), -sin(theta_rad);
     0, sin(theta_rad), cos(theta_rad)];

%% 2. Define Anchor Points and Plot the Room

% Define Anchor Points (X, Y, Z) in millimeters
anchorPoints = [
    -4795, 3200, -2220.23;
    -4795, 3200, 2748.19;
     2616.11, 3200, 2779.82;
     3011.83, 3200, -2166.38
];

% Define tolerance for rope length comparison (in millimeters)
tolerance = 10; % mm

% Apply rotation to Anchor Points
anchorPoints_rotated = (R * anchorPoints')';

% Define motor IPs
motorIps = ["192.168.0.162"; "192.168.0.116"; "192.168.0.219"; "192.168.0.182"];

% Labels for Anchor Points
labels = {'Anchor 1', 'Anchor 2', 'Anchor 3', 'Anchor 4'};

% Define Room Height (originally along Y-axis) in millimeters
roomHeight = 4000; % Example: 4000 mm height

% Calculate min and max for X and Z axes with padding
padding = 500; % mm
minX = min(anchorPoints_rotated(:,1)) - padding;
maxX = max(anchorPoints_rotated(:,1)) + padding;
minZ = min(anchorPoints_rotated(:,3)) - padding;
maxZ = max(anchorPoints_rotated(:,3)) + padding;

% Y-axis boundaries (originally Z after rotation)
minY = 0;          % Floor level
maxY = roomHeight; % Ceiling level

% Define the vertices of the room (bounding box) in rotated coordinates
roomVertices = [
    minX, minY, minZ; % 1: Front-Bottom-Left
    maxX, minY, minZ; % 2: Front-Bottom-Right
    maxX, minY, maxZ; % 3: Back-Bottom-Right
    minX, minY, maxZ; % 4: Back-Bottom-Left
    minX, maxY, minZ; % 5: Front-Top-Left
    maxX, maxY, minZ; % 6: Front-Top-Right
    maxX, maxY, maxZ; % 7: Back-Top-Right
    minX, maxY, maxZ  % 8: Back-Top-Left
];
roomVertices_rotated = (R * roomVertices')';

% Define the faces of the room
roomFaces = [
    1, 2, 3, 4; % Bottom face (Floor)
    5, 6, 7, 8; % Top face (Ceiling)
    1, 2, 6, 5; % Front face
    2, 3, 7, 6; % Right face
    3, 4, 8, 7; % Back face
    4, 1, 5, 8  % Left face
];

% Create a new figure for visualization
fig = figure('Name', 'Real-Time Motion Capture Visualization', 'NumberTitle', 'off');
hold on;
grid on;
axis equal;
xlabel('X (mm)');
ylabel('Y (mm) - Up'); % After rotation, Y is up
zlabel('Z (mm)');
title('Room Layout with Anchor Points and MCCR');

% Plot the room as a semi-transparent box
roomPatch = patch('Vertices', roomVertices_rotated, 'Faces', roomFaces, ...
      'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.1, ...
      'EdgeColor', 'k', 'DisplayName', 'Room');

% Plot Rotated Anchor Points
anchorScatter = scatter3(anchorPoints_rotated(:,1), anchorPoints_rotated(:,2), anchorPoints_rotated(:,3), ...
         100, 'filled', 'r', 'DisplayName', 'Anchors');

% Annotate Anchor Points
for i = 1:size(anchorPoints_rotated,1)
    text(anchorPoints_rotated(i,1), anchorPoints_rotated(i,2), anchorPoints_rotated(i,3), ...
         ['  ' labels{i}], 'FontSize', 12, 'Color', 'b');
end

% Optional: Highlight the plane where anchor points lie (after rotation, Y is up)
[X_grid, Z_grid] = meshgrid(linspace(minX, maxX, 2), linspace(minZ, maxZ, 2));
Y_grid = ones(size(X_grid)) * anchorPoints_rotated(1,2); % Y = 3200 mm
anchorPlane = surf(X_grid, Y_grid, Z_grid, 'FaceColor', 'yellow', ...
                  'FaceAlpha', 0.05, 'EdgeColor', 'none', 'DisplayName', 'Anchor Plane');

% Adjust view angle for better visualization
view(3);            % 3D view
rotate3d on;        % Enable interactive rotation
camlight headlight; % Add a light source for better shading
lighting gouraud;   % Use Gouraud lighting

%% 3. Initialize NatNet Client and Define Rigid Body

% Initialize NatNet client
natnetclient = initializeNatNet(); % Ensure this function is defined and works

% Define the name of the rigid body to track
rigidBodyName = 'MCCR';

% Verify if the rigid body exists in the model description
model = natnetclient.getModelDescription;
rigidBodyExists = false;
for i = 1:model.RigidBodyCount
    if strcmp(model.RigidBody(i).Name, rigidBodyName)
        rigidBodyExists = true;
        break;
    end
end
if ~rigidBodyExists
    error('Rigid body "%s" not found in the model description.', rigidBodyName);
end

%% 4. Calculate Rope Offsets

% Retrieve the initial position of the rigid body to calculate ideal rope lengths
initialBodyPosition = getBodyPosition(natnetclient);
if any(isnan(initialBodyPosition))
    error('Initial rigid body position not available.');
end

% Apply rotation to the initial position
initialBodyPosition_rotated = (R * initialBodyPosition')';

% Calculate ideal rope lengths based on initial position
idealRopeLengths = [norm(initialBodyPosition_rotated - anchorPoints_rotated(1, :)), ...
                    norm(initialBodyPosition_rotated - anchorPoints_rotated(2, :)), ...
                    norm(initialBodyPosition_rotated - anchorPoints_rotated(3, :)), ...
                    norm(initialBodyPosition_rotated - anchorPoints_rotated(4, :))];

% Retrieve real rope lengths from encoders
realRopeLengths = [getEncoderData(motorIps(1)).ropeLength, ...
                   getEncoderData(motorIps(2)).ropeLength, ...
                   getEncoderData(motorIps(3)).ropeLength, ...
                   getEncoderData(motorIps(4)).ropeLength];

% Calculate rope offsets
ropeOffset = realRopeLengths - idealRopeLengths;

%% 5. Initialize 'MCCR' Marker and Ropes on the Plot

% Initialize 'MCCR' scatter plot as a blue filled sphere, initially not visible
mccrScatter = scatter3(NaN, NaN, NaN, 150, 'filled', 'b', 'DisplayName', 'MCCR');

% Annotate 'MCCR' with a text label, initially not visible
mccrText = text(NaN, NaN, NaN, 'MCCR', 'FontSize', 12, 'Color', 'g');

% Initialize a trail for 'MCCR' (optional)
trailLength = 100; % Number of previous positions to display
trailX = NaN(1, trailLength);
trailY = NaN(1, trailLength);
trailZ = NaN(1, trailLength);
trailPlot = plot3(trailX, trailY, trailZ, 'b-', 'LineWidth', 1, 'DisplayName', 'MCCR Trail');

% Initialize Line Objects for Ropes
ropeLines = gobjects(4,1); % Preallocate for 4 ropes
for i = 1:4
    ropeLines(i) = line('XData', [anchorPoints_rotated(i,1), NaN], ...
                        'YData', [anchorPoints_rotated(i,2), NaN], ...
                        'ZData', [anchorPoints_rotated(i,3), NaN], ...
                        'LineWidth', 2, 'Color', 'k', 'DisplayName', ['Rope ' num2str(i)]);
end

% Initialize Text Objects for Rope Length Labels
ropeTexts = gobjects(4,1); % Preallocate for 4 rope labels
for i = 1:4
    ropeTexts(i) = text(NaN, NaN, NaN, '', 'FontSize', 10, 'Color', 'k', 'BackgroundColor', 'w');
end

%% 6. Initialize Shared Data for 'MCCR' Position

% Initialize shared data storage for 'position_rotated' using appdata
setappdata(fig, 'position_rotated', [NaN, NaN, NaN]);

%% 7. Setup Timer for Asynchronous Rope Updates

% Create a timer object that executes every 60 seconds (1 minute)
ropeTimer = timer(...
    'ExecutionMode', 'fixedRate', ...  % Run the timer repeatedly
    'Period', 60, ...                  % Time in seconds between executions
    'TimerFcn', @updateRopesCallback ...% Callback function to execute
);

% Start the timer
start(ropeTimer);

% Ensure that the timer is deleted when the figure is closed
cleanupObj = onCleanup(@() cleanupTimer(ropeTimer));

%% 8. Real-Time Tracking Loop

% Initialize a flag to indicate if MCCR is currently being tracked
isTracked = false;

% Start real-time plotting
disp('Starting real-time MCCR tracking. Press Ctrl+C to stop.');

% Use a try-catch block to allow graceful termination
try
    while true
        % Check connection status
        if ~natnetclient.IsConnected
            warning('NatNet client disconnected. Attempting to reconnect...');
            natnetclient = initializeNatNet(); % Reinitialize the client
            pause(1); % Wait before retrying
            continue;
        end

        % Retrieve position data
        position = getBodyPosition(natnetclient); % Ensure this function returns [X Y Z]
        % disp(position); % Uncomment for debugging
        
        % Apply rotation to the retrieved position
        position_rotated = (R * position')';

        if ~any(isnan(position_rotated)) % Data is available
            % Update 'MCCR' scatter plot
            set(mccrScatter, 'XData', position_rotated(1), 'YData', position_rotated(2), 'ZData', position_rotated(3));

            % Update 'MCCR' text annotation
            set(mccrText, 'Position', position_rotated, 'String', 'MCCR');

            % Update trail data
            trailX = [trailX(2:end), position_rotated(1)];
            trailY = [trailY(2:end), position_rotated(2)];
            trailZ = [trailZ(2:end), position_rotated(3)];
            set(trailPlot, 'XData', trailX, 'YData', trailY, 'ZData', trailZ);

            % Store the current position in appdata for the timer to access
            setappdata(fig, 'position_rotated', position_rotated);

            % Ensure 'MCCR' and trail are visible
            if ~isTracked
                set(mccrScatter, 'Visible', 'on');
                set(mccrText, 'Visible', 'on');
                set(trailPlot, 'Visible', 'on');
                isTracked = true;
            end
        else
            % No data available, hide 'MCCR' marker and trail
            if isTracked
                set(mccrScatter, 'Visible', 'off');
                set(mccrText, 'Visible', 'off');
                set(trailPlot, 'Visible', 'off');
                isTracked = false;
            end
        end

        % Update the plot
        drawnow;

        % Pause for a short duration to limit update rate (e.g., 20 Hz)
        pause(0.05); % 50 ms
    end
catch ME
    % Stop and delete the timer in case of any errors or interruptions
    stop(ropeTimer);
    delete(ropeTimer);

    disp('Real-time tracking stopped.');
    disp(ME.message);
end

%% 9. Timer Callback Function for Rope Updates

function updateRopesCallback(~, ~)
    % Callback function to update ropes asynchronously every minute

    % Retrieve the current position_rotated from appdata
    position_rotated = getappdata(gcf, 'position_rotated');

    if any(isnan(position_rotated))
        warning('Rope update skipped: Current MCCR position is unavailable.');
        return;
    end

    % Retrieve real rope lengths from encoders
    realRopeLengths = [getEncoderData(motorIps(1)).ropeLength, ...
                       getEncoderData(motorIps(2)).ropeLength, ...
                       getEncoderData(motorIps(3)).ropeLength, ...
                       getEncoderData(motorIps(4)).ropeLength];

    % Calculate adjusted rope lengths
    adjustedRopeLengths = realRopeLengths - ropeOffset;

    % Calculate expected rope lengths based on current position
    expectedRopeLengths = [
        norm(position_rotated - anchorPoints_rotated(1, :)), ...
        norm(position_rotated - anchorPoints_rotated(2, :)), ...
        norm(position_rotated - anchorPoints_rotated(3, :)), ...
        norm(position_rotated - anchorPoints_rotated(4, :))
    ];

    % Update Rope Lines and Labels
    for i = 1:4
        % Define the end point of the rope line (MCCR position)
        ropeEnd = position_rotated;

        % Update the line data
        set(ropeLines(i), 'XData', [anchorPoints_rotated(i,1), ropeEnd(1)], ...
                         'YData', [anchorPoints_rotated(i,2), ropeEnd(2)], ...
                         'ZData', [anchorPoints_rotated(i,3), ropeEnd(3)]);

        % Calculate the midpoint for placing the text label
        midpoint = (anchorPoints_rotated(i, :) + ropeEnd) / 2;

        % Update the text label with adjusted rope length
        set(ropeTexts(i), 'Position', midpoint, ...
                         'String', sprintf('%.1f mm', adjustedRopeLengths(i)), ...
                         'HorizontalAlignment', 'center', ...
                         'VerticalAlignment', 'bottom');

        % Check if adjusted rope length matches expected rope length within tolerance
        if abs(adjustedRopeLengths(i) - expectedRopeLengths(i)) > tolerance
            % Log a warning message
            fprintf('Warning: Rope %d length mismatch. Expected: %.1f mm, Adjusted: %.1f mm\n', ...
                    i, expectedRopeLengths(i), adjustedRopeLengths(i));

            % Change rope line color to red to indicate discrepancy
            set(ropeLines(i), 'Color', 'r');

            % Optionally, change the text color to red
            set(ropeTexts(i), 'Color', 'r');
        else
            % Reset rope line color to black if within tolerance
            set(ropeLines(i), 'Color', 'k');

            % Reset the text color to black
            set(ropeTexts(i), 'Color', 'k');
        end
    end
end

%% 10. Cleanup Function to Stop and Delete Timer

function cleanupTimer(t)
    % Cleanup function to stop and delete the timer
    try
        stop(t);
        delete(t);
    catch
        % Ignore errors during cleanup
    end
end

%% 7. Supporting Functions

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