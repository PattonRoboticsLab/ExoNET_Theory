function [tauvect, T, Tdist, tension] = tauMARIONETBUNJEE( shoulder, endpoint, Actual_Pin, l0 ) 
    
    %% Tension Function for Rohith's Eighth Inch Bungee Cords
    %tension = @(L0,L) (112.2*(L/L0).^5-838.3*(L/L0).^4+2494*(L/L0).^3-3689*(L/L0).^2+2717*(L/L0)-794.2).*((L/L0)>1);
    %% Tension for Yaseen's Quarter Inch Bungee Cords
    %tension = @(L0,L) ((710.5*(L/L0).^5-5442*(L/L0).^4+1.654e4*(L/L0).^3-2.495e4*(L/L0).^2+1.871e4*(L/L0)-5575)).*((L/L0)>1);
    %tension = @(L0,L) (Exo.K.*(L-L0)).*((L-L0)>0);   % (inlineFcn) +Stretch
    %% Tension Quarter Inch Maroon_Teal Bungee
    %tension = @(L0,L) (370.9*(L/L0).^5-2963*(L/L0).^4+9398*(L/L0).^3-14790*(L/L0).^2+11590*(L/L0)-3600).*((L/L0)>1);
    
    %% Tan rubber band
    % tension = @(lambda) (8700498006740773*lambda^6)/36028797018963968 - (5759379262620855*lambda^5)/2251799813685248 +...
    %     (5465647199008601*lambda^4)/562949953421312 - (3358409318379815*lambda^3)/281474976710656 - (2118513679857703*lambda^2)/140737488355328 + (8132963846634919*lambda)/140737488355328 - 2687841211882243/70368744177664;
    
    %% Black rubber band
    % tension = @(lambda) - (8313168292301631*lambda^6)/9007199254740992 + (2003599846877743*lambda^5)/140737488355328 -...
    %     (6343646839143871*lambda^4)/70368744177664 + (5337009743429569*lambda^3)/17592186044416 - (2539290890064221*lambda^2)/4398046511104 + (5380556851910277*lambda)/8796093022208 - 4589261501246089/17592186044416;
    
    %% Red rubber band
    % tension = @(lambda) - (7071771297026501*lambda^6)/4503599627370496 + (1472586864503473*lambda^5)/70368744177664 - (8154478902530947*lambda^4)/70368744177664 +...
    %     (377106218631621*lambda^3)/1099511627776 - (79089566353517*lambda^2)/137438953472 + (2345521126831727*lambda)/4398046511104 - 7188444792093581/35184372088832;
    
    %% Green rubber band
    tension = @(lambda) - (8953556697816443*lambda^6)/4503599627370496 + (7266562633001257*lambda^5)/281474976710656 - (2470409444250189*lambda^4)/17592186044416 + (7241353931205205*lambda^3)/17592186044416 -...
        (6065393101531663*lambda^2)/8796093022208 + (352537180651245*lambda)/549755813888 - 2170266699221631/8796093022208;
    
    %% Yaseen
    %tension = @(lambda) ((710.5*(lambda).^5-5442*(lambda).^4+1.654e4*(lambda).^3-2.495e4*(lambda).^2+1.871e4*(lambda)-5575)).*((lambda)>1);

    % L_pre_extended_recoil is the length of the spring that remains fixed
    % in order to set a threshold of force
    rVect = endpoint - shoulder;  lVect = Actual_Pin - shoulder;        
     Tdir = lVect - rVect;        Tdist = sqrt(sum(Tdir.^2, 2));  
    
    Tdir_unit  = Tdir ./ Tdist;

    T = tension( Tdist / l0 );  % Uses Inline Function for Tension in Setup.m
       
    tauvect = cross(rVect, T .* Tdir_unit);

end % end function