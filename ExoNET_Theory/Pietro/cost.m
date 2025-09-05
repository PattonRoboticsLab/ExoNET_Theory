% ***********************************************************************
% Evaluate the cost function for desired torques TAUs at positions PHIs 
% ***********************************************************************
function [c, gradc] = cost(p, TAUsDesired, Exo, Pos, Bod, robot, q)

% CostOnly evaluate the cost
c = costOnly(p, TAUsDesired, Exo, Pos, Bod, robot, q);

% ApproxGrad evaluate an approximation of the gradient of the function to
% accelerate fmincon function
gradc = approxGrad(p, @(pp) costOnly(pp, TAUsDesired, Exo, Pos, Bod, robot, q), c);

%% Code below here may be helpful for other stuff like regolarization
% nparams = Exo.nParamsSh * round( Exo.numbconstraints(2) );    
% p_wr = p(end-2:end);  p_sw = p( 1:nparams+1 );  p_el = p( nparams+2:nparams*2+2 );
% e1 = e_el(:,1);     e2 = e_el(:,2);     e3 = e_el(:,3);
% e_Shoulder = TAUsDesired.TauSh_tot - TauExo.elevationSh;
% e_Elbow = TAUsDesired.TauEl_tot - TauExo.elevationEl;
% 
% c = sum(e_Shoulder(:).^2) + sum(e_Elbow(:).^2);
% e_sh = TAUsDesired.TauEl_tot - TauExo.elevationEl ;
% e4 = e_sh(:,1);     e5 = e_sh(:,2);     e6 = e_sh(:,3);
% 
% % Store the error between the two forces
% Exo.E1 = e1;   Exo.E2 = e2;   Exo.E3 = e3;        
% Exo.E5 = e5;   Exo.E4 = e4;   Exo.E6 = e6;  
% 
% sum_err1 = sum(e1.^2);    sum_err2 = sum(e2.^2);    sum_err3 = sum(e3.^2);
% sum_err4 = sum(e4.^2);    sum_err5 = sum(e5.^2);    sum_err6 = sum(e6.^2);  
% 
% % Calculate the final cost summing the squares of the errors at all positions
% c = sum_err1 + sum_err2 + sum_err3 + sum_err4 + sum_err5 + sum_err6;


end