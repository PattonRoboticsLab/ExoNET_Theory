function q = estimateJointAnglesFromPositions(robot, shoulder, elbow, wrist)

    % Posizioni relative rispetto alla spalla
    elbow_rel = elbow - shoulder;
    wrist_rel = wrist - shoulder;

    % Inizializza il solver di cinematica inversa generalizzata
    gik = generalizedInverseKinematics(...
        'RigidBodyTree', robot, ...
        'ConstraintInputs', {'position', 'position'});

    % Vincolo posizione gomito (rigid body: 'upper_arm')
    posElbow = constraintPositionTarget('upper_arm');
    posElbow.TargetPosition = elbow_rel;
    posElbow.PositionTolerance = 0.01;

    % Vincolo posizione polso (rigid body: 'forearm')
    posWrist = constraintPositionTarget('forearm');
    posWrist.TargetPosition = wrist_rel;
    posWrist.PositionTolerance = 0.01;

    % Configurazione iniziale
    initialguess = robot.homeConfiguration;

    % Risolvi
    [q, solInfo] = gik(posElbow, posWrist, initialguess);

    % DEBUG opzionale: verifica errore effettivo
    T_elbow = getTransform(robot, q, 'upper_arm');
    T_wrist = getTransform(robot, q, 'forearm');
    err_elbow = norm(tform2trvec(T_elbow) - elbow_rel);
    err_wrist = norm(tform2trvec(T_wrist) - wrist_rel);

    if err_elbow > 0.02 || err_wrist > 0.02
        warning('Errore IK alto: elbow = %.2f cm, wrist = %.2f cm', ...
            err_elbow * 100, err_wrist * 100);
    end
end
