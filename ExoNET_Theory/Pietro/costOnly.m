% Cost function to evaluate only the cost of the cost function, used in
% approxGrad function to accelerate the process and accelerate the code
function c = costOnly(p, TAUsDesired, Exo, Pos, Bod, robot, q)

    % Call exoNetTorque3D to evaluate the torque given from the exoNet
    [TauExo, ~, ~, ~, ~, ~] = exoNetTorques3D(Pos, Bod, Exo, p, robot, q); 
    
    % Evaluation of the error
    eSh = TAUsDesired.TauSh_tot - TauExo.elevationSh;
    eEl = TAUsDesired.TauEl_tot - TauExo.elevationEl;

    c = sum(eSh(:).^2) + sum(eEl(:).^2);

    %% Fix P vector
    nparams = Exo.nParamsSh * round( Exo.numbconstraints(2) ); 

        p_wr = p( end-2 : end );
    p_el(:,1) = p( 1 : nparams+1 );
    p_sw(:,1) = p( nparams+2 : nparams*2+2 );
    
   %% Regularization for p_sw
    if round( p_sw(1) ) > 1
        c = c + 20; 
    end

    %% Regularization for p_el
    if round( p_el(1) ) > 1
        c = c + 20; 
    end

end