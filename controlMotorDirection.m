function success = controlMotorDirection(motorIp, direction)
    % controlMotorDirection - Commands the motor to release or pull the rope.
    %
    % Syntax: success = controlMotorDirection(motorIp, direction)
    %
    % Inputs:
    %   motorIp  - String specifying the motor's IP address (e.g., '192.168.0.162')
    %   direction - String indicating the direction ('release' or 'pull')
    %
    % Outputs:
    %   success - Boolean indicating whether the direction command was successfully sent

    % Validate the direction input
    validDirections = {'release', 'pull', 'stop'};
    if ~ismember(lower(direction), validDirections)
        error('Invalid direction. Use "release", "pull" or "stop".');
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
            warning('Motor %s responded with error: %s', motorIp, response.message);
            success = false;
        end
    catch ME
        % Handle any errors during the HTTP request
        warning('Failed to set direction for motor %s: %s', motorIp, ME.message);
        success = false;
    end
end
