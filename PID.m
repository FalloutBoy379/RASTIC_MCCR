function [vel_output, error] = PID(targetPosition, currentPosition, last_error, dt, Kp, Ki, Kd)
    % PID - Computes velocity output for pendant based on position error
    %
    % Inputs:
    %   targetPosition - Desired target position [x, y, z]
    %   currentPosition - Current pendant position [x, y, z]
    %   last_error - Last error value [x, y, z]
    %   dt - Time step for integration and differentiation
    %   Kp, Ki, Kd - PID gains
    %
    % Outputs:
    %   vel_output - Velocity vector [vx, vy, vz]
    %   error - Current error [x, y, z]
    
    error = targetPosition - currentPosition; % Position error
    persistent integral_error;
    
    if isempty(integral_error)
        integral_error = [0, 0, 0]; % Initialize integral term
    end
    
    % PID terms
    P = Kp * error; % Proportional term
    integral_error = integral_error + error * dt; % Integral term
    I = Ki * integral_error;
    D = Kd * (error - last_error) / dt; % Derivative term
    
    vel_output = P + I + D; % Velocity output
end
