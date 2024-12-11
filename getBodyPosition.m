function position = getBodyPosition(natnetclient)
    position = getRigidBodyPosition(natnetclient);
end

function position = getRigidBodyPosition(natnetclient)
    % Get the model description
    model = natnetclient.getModelDescription;
    

    % Find the rigid body index
    rigidBodyIndex = 2;
    mrIndex = 1;

    mdata = natnetclient.getFrame;

    while mdata.RigidBodies(mrIndex).ID ~= rigidBodyIndex
        mrIndex = mrIndex +1;
    end

    markerNumber = mrIndex;



    % Get the current frame data
    data = natnetclient.getFrame;
    if data.RigidBodies(rigidBodyIndex).Tracked
        % Extract position
        position = [
            data.RigidBodies(markerNumber).x * 1000, ... % X in mm
            data.RigidBodies(markerNumber).y * 1000, ... % Y in mm
            data.RigidBodies(markerNumber).z * 1000 ... % Z in mm
        ];
    else
        error('Rigid body "%s" is not currently tracked.', rigidBodyName);
    end
end