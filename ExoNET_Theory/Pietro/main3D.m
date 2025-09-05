% Main: main script to do exoNet    
% Patton's main program. use the other main to do Carella's
%% Begin
clear; close all; clc; 
fprintf('\n ~ MAIN script:  ~ \n')  
% setUp3D % set most variables and plots in a SCRIPT y
setUp3D
switch ProjectName    
    
    case 1 % Gravity Compensation robustOpto3D_optimized
        [ TAUsDesired, MaxTorques, robot, q ] = weightEffect3D( Bod, Pos, Exo); 
        [ Actual_Pin, bestP_3D, bestCost, TauExo,  L_springs, L0, Tension, Force_vector] = robustOpto3D_opt( Bod, Pos, Exo, PHIs, TAUsDesired, robot, q);       
        [TauResidual, RMSE_exo] = Residualtorque( TauExo, TAUsDesired, Pos, Bod, Exo, Tension, bestCost, bestP_3D ); 
        SaveResults( TauExo, TauResidual, TAUsDesired, Actual_Pin, bestCost, RMSE_exo, bestP_3D, Force_vector );

    case 2 % Evaluation Exo and also possible to see the animation with Plotanimarion
        bestP_3D = loadBestP3D();    results = EvaluationExoExpData(Bod, Exo, bestP_3D);
   
    case 3 % Error Augmentation 
        results = EAanalysis(Bod);   results.type = 'healthy';  % 'healthy' or 'stroke'
        AnalyzeResults(results);
 
    case 4 % Evaluation of the forces 
        [p_vector, Torque_Exo, Torque_desired, Force_vector] = loadTorques();   
        Force_Analysis_exo_opt(p_vector, Exo, Bod, Pos, Torque_Exo, Torque_desired, Force_vector)
    
    otherwise
        disp('exiting..'); close all
    
end % END switch

fprintf(' end MAIN script. \n')