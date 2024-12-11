function motorVelocities = solveKinematics(anchorPoints, currentBodyPosition, velocity)
    % solveKinematics - Maps pendant velocity to motor velocities
    %
    % Inputs:
    %   anchorPoints - [4x3] matrix of anchor positions
    %   currentBodyPosition - [1x3] current pendant position [x, y, z]
    %   velocity - [1x3] pendant velocity vector [vx, vy, vz]
    %
    % Outputs:
    %   motorVelocities - [1x4] vector of motor rope velocities

    %% Compute unit vectors along each rope
    % Instantiate empty vector
    numMotors = size(anchorPoints, 1);
    unitVectors = zeros(numMotors, 3);

    % Iterate over each motor
    for i = 1:numMotors
        
        ropeVector = currentBodyPosition - anchorPoints(i, :);
        unitVectors(i, :) = ropeVector / norm(ropeVector);
    end

    % Map pendant velocity to motor rope velocities
    motorVelocities = unitVectors * velocity(:); % Project velocity along rope directions
end
