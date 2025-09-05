function [torque, acceleration, velocity ] = ShoulderDynamics(Body, time, Bod, filename)

    %% Detect affected arm from filename
    if contains(lower(filename), 'left')
        armSide = 'l';
    elseif contains(lower(filename), 'right')
        armSide = 'r';
    else
        error('Filename must contain "Left" or "Right" to detect affected side.');
    end
    
    %% Run inverse dynamics
    [velocityOut, accelerationOut, torqueOut] = processArm(Body, time, Bod, armSide, filename);

    %% Assign all outputs
    torque = torqueOut;
    velocity = velocityOut;
    acceleration = accelerationOut;

end
