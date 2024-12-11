function encoderData = getEncoderData(arduinoIP)
    % getEncoderData Fetches encoder data from the Arduino web server
    %
    % Inputs:
    %   arduinoIP - String, IP address of the Arduino (e.g., '192.168.0.185')
    %
    % Outputs:
    %   encoderData - Struct containing yaw, rope, ropeLength, and ropeOffset

    % Construct the URL for encoder data
    % arduinoIP
    url = sprintf('http://%s/encoders', arduinoIP);
    
    % Use webread to fetch the JSON data
    options = weboptions('Timeout', 5); % Set timeout to 5 seconds
    try
        jsonData = webread(url, options);
        encoderData.yaw = jsonData.yaw;
        encoderData.rope = jsonData.rope;
        encoderData.ropeLength = jsonData.ropeLength;
    catch ME
        warning('Failed to retrieve encoder data from %s: %s', arduinoIP, ME.message);
        encoderData = struct('yaw', NaN, 'rope', NaN, 'ropeLength', NaN);
    end
end
