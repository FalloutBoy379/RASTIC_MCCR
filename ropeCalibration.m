anchorPoints = [
    -4795, 3200, -2220.23;
    -4795, 3200, 2748.19;
     2616.11, 3200, 2779.82;
     3011.83, 3200, -2166.38
];

motorIps = ["192.168.0.162";"192.168.0.116"; "192.168.0.219"; "192.168.0.182"];

natnet = initializeNatNet();
currentBodyPosition = getBodyPosition(natnet);
idealRopeLengths = [norm(currentBodyPosition - anchorPoints(1, :)), norm(currentBodyPosition - anchorPoints(2, :)), norm(currentBodyPosition - anchorPoints(3, :)), norm(currentBodyPosition - anchorPoints(4, :))]
realRopeLengths = [getEncoderData(motorIps(1)).ropeLength, getEncoderData(motorIps(2)).ropeLength, getEncoderData(motorIps(3)).ropeLength, getEncoderData(motorIps(4)).ropeLength]

ropeOffset = idealRopeLengths - realRopeLengths

function natnetclient = initializeNatNet()
    natnetclient = natnet;
    natnetclient.HostIP = '192.168.0.103'; % Adjust to OptiTrack server IP
    natnetclient.ClientIP = '192.168.0.185'; % Adjust to your local IP
    natnetclient.ConnectionType = 'Multicast';
    natnetclient.connect;
    if ~natnetclient.IsConnected
        error('Failed to connect to NatNet server.');
    end
end