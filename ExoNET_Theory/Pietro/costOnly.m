% Cost function to evaluate only the cost of the cost function, used in
% approxGrad function to accelerate the process and accelerate the code
function c = costOnly(p, TAUsDesired, Exo, Pos, Bod)

    % Call exoNetTorque3D to evaluate the torque given from the exoNet
    TauExo = exoNetTorques3D(Pos, Bod, Exo, p); 
    
    % Evaluation of the error
    eSh = TAUsDesired.TauSh_tot - TauExo.elevationSh;
    eEl = TAUsDesired.TauEl_tot - TauExo.elevationEl;
    
    alpha = 1; beta = 1; % Factor to give more importance to which torque

    c = sum( alpha * eSh(:).^2 ) + sum( beta * eEl(:).^2 );

    %% Fix P vector
    nparams = Exo.nParamsSh * round( Exo.numbconstraints(2) ); 
 
    p_el(:,1) = p( 1 : nparams+1 );   p_sw(:,1) = p( nparams+2 : nparams*2+2 );
    %p_wr = p( end-2 : end );
    
    %% Increase the cost for the extended arm to have more stability on this position
    % gamma = 4;
    % Tau_desired_sh_firstplane = vecnorm( TAUsDesired.TauSh_tot(1:2:end,:), 2, 2 );
    % Tau_exo_sh_firstplane = vecnorm( TauExo.elevationSh(1:2:end,:), 2, 2 );
    % 
    % c = c + gamma * sum((Tau_desired_sh_firstplane - Tau_exo_sh_firstplane).^2);

    %% Increase the cost for the flexed arm to have more stability on this position
    % delta = 4;
    % Tau_desired_sh_secondplane = vecnorm( TAUsDesired.TauSh_tot(2:2:end,:), 2, 2 );
    % Tau_exo_sh_secondplane = vecnorm( TauExo.elevationSh(2:2:end,:), 2, 2 );
    % 
    % c = c + delta * sum((Tau_desired_sh_secondplane - Tau_exo_sh_secondplane).^2);

    %% Regularization for p_sw to have less components and be a real solution 
    % if round( p_sw(1) ) > 2
    %     c = c + 5 *round( p_sw(1) ); 
    % end
    % 
    % %% Regularization for p_el
    % if round( p_el(1) ) > 2
    %     c = c + 5 *round( p_el(1) ); 
    % end

end