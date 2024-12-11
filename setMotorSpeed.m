function success = setMotorSpeed(motorIp, speed_f)
    % setMotorSpeed - Sets the speed of the motor at the specified IP address.
    %
    % Syntax: success = setMotorSpeed(motorIp, speed)
    %
    % Inputs:
    %   motorIp - String specifying the motor's IP address (e.g., '192.168.0.162')
    %   speed   - Numeric value representing the desired speed (units depend on motor API)
    %
    % Outputs:
    %   success - Boolean indicating whether the speed was successfully set
    
    try
        speed = int32(speed_f);
        if(speed == 0)

            controlMotorDirection(motorIp, 'stop');
        else
        % Construct the URL based on the motor's API endpoint for setting speed
        url = sprintf('http://%s/setSpeed?speed=%d', motorIp, speed);
        
        % Send the HTTP GET request to set the speed
        response = webread(url);
        
        % % Check if the response indicates success (modify based on actual API response)
        % if contains(response, 'OK', 'IgnoreCase', true)
        %     success = true;
        % else
        %     warning('Motor %s responded with error: %s', motorIp, response.message);
        %     success = false;
        % end
        end
    catch ME
        % Handle any errors during the HTTP request
        warning('Failed to set speed for motor %s: %s', motorIp, ME.message);
        success = false;
    end
end
