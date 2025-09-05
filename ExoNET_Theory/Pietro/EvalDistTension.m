%% Function to evaluate the torques of the exo 
function [Tdist] = EvalDistTension( endpoint, Actual_Pin)
    
    rVect  = endpoint; 
    lVect  = Actual_Pin;    
    
    Tdir  = lVect - rVect;  
    Tdist = norm( Tdir );  

end % end function