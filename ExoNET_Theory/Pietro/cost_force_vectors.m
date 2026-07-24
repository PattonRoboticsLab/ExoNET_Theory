function c = cost_force_vectors(p, TAUsDesired, Exo, Pos, Bod,robot, q, Force_flexed, Force_extended)

    TauExo = exoNetTorques3D(Pos, Bod, Exo, p); 
    
    Force_exo_extended = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TauExo.elevationSh, TauExo.elevationEl, ...
                                    [0, 0, 1], 1.5,   4, 1, 'Transparency', 0.7, 'NoPlot', true);
    Force_exo_flexed = plotVectField3D_opt(q, Bod, Pos, Exo, robot, TauExo.elevationSh, TauExo.elevationEl, ...
                    [0, 0, 1], 1.5,   4, 2, 'Transparency', 0.7, 'NoPlot', true);
    
    err_ext  = Force_extended - Force_exo_extended;   % [N x 3]
    err_flex = Force_flexed   - Force_exo_flexed;     % [N x 3]

    e_ext  = vecnorm(err_ext,  2, 2);   % [N x 1]
    e_flex = vecnorm(err_flex, 2, 2);   % [N x 1]

        alpha = 1; beta = 1; % Factor to give more importance to which torque

    c = alpha * sum(e_ext.^2) + beta * sum(e_flex.^2);

end