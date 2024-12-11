anchorPoints = [
    -4795, 3200, -2220.23;
    -4795, 3200, 2748.19;
     2616.11, 3200, 2779.82;
     3011.83, 3200, -2166.38
];

motorIps = ["192.168.0.162"; "192.168.0.116"; "192.168.0.219"; "192.168.0.182"];

%% 
natnetclient = initializeNatNet();

initialBodyPosition = getBodyPosition(natnetclient);
if any(isnan(initialBodyPosition))
    error('Initial rigid body position not available.');
end


idealRopeLengths = [norm(initialBodyPosition - anchorPoints(1,:)), ...
                    norm(initialBodyPosition - anchorPoints(2,:)), ...
                    norm(initialBodyPosition - anchorPoints(3,:)), ...
                    norm(initialBodyPosition - anchorPoints(4,:))];

realRopeLengths = [getEncoderData(motorIps(1)).ropeLength, ...
                    getEncoderData(motorIps(2)).ropeLength, ...
                    getEncoderData(motorIps(3)).ropeLength, ...
                    getEncoderData(motorIps(4)).ropeLength, ];

ropeOffset = realRopeLengths - idealRopeLengths;


targetPosition = initialBodyPosition - [30,30,30];

setMotorSpeed(motorIps(1), 100);
controlMotorDirection(motorIps(1), 'pull');
pause(0.2)
controlMotorDirection(motorIps(1), 'stop');








% function [vel_output, error] = PID(target, current, last_error)
%     error = target - current;
% 
% 
%     P = error * KP;
%     I = 
% end



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