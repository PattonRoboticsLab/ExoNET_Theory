%% Function to evaluate the torque of the exo in the space 
function [TauExo, Actual_Pin, L_springs, L0, Tension, func_tension] = exoNetTorques3D(Pos, Bod, Exo, p_init)

    %% Evaluates numbers of parameters and params per joint
    nparams = Exo.nParamsSh * round( Exo.numbconstraints(2) ); 
    
    %% Fix P vector
            p_wr = p_init( end-2 : end );
       p_el(:,1) = p_init( 1 : nparams+1 );
       p_sw(:,1) = p_init( nparams+2 : nparams*2+2 );
    
    %% Set the pins, L0 and endpoint
          numbsh = round( p_el(1) );                          numbswivel = round( p_sw(1) );    
    l0_elevation = zeros(1, numbsh);                           l0_swivel = zeros(1, numbswivel);
     L_elevation = zeros(size(Pos.elbowSwivel,1), numbsh);      L_swivel = zeros(size(Pos.elbowSwivel,1), numbswivel );
     T_elevation = zeros(size(Pos.elbowSwivel,1), numbsh);      T_swivel = zeros(size(Pos.elbowSwivel,1), numbswivel );
T_dist_elevation = zeros(size(Pos.elbowSwivel,1), numbsh); T_dist_swivel = zeros(size(Pos.elbowSwivel,1), numbswivel);
    
    %% Create Pins for evaluation
    Actual_Pin_elevation = zeros(numbsh,3);  
    for i = 1:numbsh
        Actual_Pin_elevation(i,:) = Pos.sh + [ p_el( 4 + Exo.nParamsSh*(i-1)), ...
                                    p_el( 2 + Exo.nParamsSh*(i-1)) * cos( p_el( 3 + Exo.nParamsSh*(i-1)) ), ...
                                    p_el( 2 + Exo.nParamsSh*(i-1)) * sin( p_el( 3 + Exo.nParamsSh*(i-1)) ) ]; 
               
        %  l0_recoil_el(i) = Exo.L_rail / Exo.stretch_ratio_limit_recoil - (Exo.L_rail / Exo.stretch_ratio_limit_recoil) * p_el( 9 + Exo.nParamsSh*(i-1));
        % 
        %  L0_pre_ext_el(i) = Exo.L_rail * p_el( 10 + Exo.nParamsSh*(i-1));
        
    end
    
    Actual_Pin_swivel = zeros(numbswivel,3);  
    for i = 1:numbswivel   
        Actual_Pin_swivel(i,:) = Pos.sh + [ p_sw( 4 + Exo.nParamsSh*(i-1)), ...
                                 p_sw( 2 + Exo.nParamsSh*(i-1)) * cos( p_sw( 3 + Exo.nParamsSh*(i-1)) ), ...
                                 p_sw( 2 + Exo.nParamsSh*(i-1)) * sin( p_sw( 3 + Exo.nParamsSh*(i-1)) ) ];    
    

        %  l0_recoil_sw(i) = Exo.L_rail / Exo.stretch_ratio_limit_recoil - (Exo.L_rail / Exo.stretch_ratio_limit_recoil) * p_sw( 9 + Exo.nParamsSh*(i-1));
        % 
        %  L0_pre_ext_sw(i) = Exo.L_rail * p_sw( 10 + Exo.nParamsSh*(i-1));
    end
    
    %% Find the maximum length of the springs
    for j = 1:size(Pos.elbowSwivel,1) 
        for i = 1:numbsh  
            L_elevation(j,i) = norm( Pos.gapel(j,:) - Actual_Pin_elevation(i,:) );  
        end 
        for i = 1:numbswivel    
            L_swivel(j,i) = norm( Pos.gapsw(j,:) - Actual_Pin_swivel(i,:) );
        end
    end
    maxElevation = max(L_elevation,[], 1);   maxSwivel = max(L_swivel,[], 1);

    %% find the resting lengths
    for i = 1:numbsh
         l0_elevation(i) = maxElevation(i) / Exo.stretch_ratio_limit + maxElevation(i) / Exo.stretch_ratio_limit  * p_el( 5 + Exo.nParamsSh*(i-1));
    end
    for i = 1:numbswivel
        l0_swivel(i) = maxSwivel(i) / Exo.stretch_ratio_limit + maxSwivel(i) / Exo.stretch_ratio_limit  * p_sw( 5 + Exo.nParamsSh*(i-1));
    end
    l0_wrist = ( Bod.L(2) + Bod.L(1) ) / Exo.stretch_ratio_limit + ( Bod.L(2) + Bod.L(1) ) * p_wr( 1 ); % Divided by four to see if it's working
    
    %% Evaluate the max lengths
    % for i = 1:numbsh  
    %     maxDist_elevation = max(L_elevation, [], 1);    minDist_elevation = min(L_elevation, [], 1);
    %     DLmax_elevation = maxDist_elevation - minDist_elevation;
    % end    
    % for i = 1:numbswivel    
    %     maxDist_swivel = max(L_swivel, [], 1);          minDist_swivel = min(L_swivel, [], 1);
    %     DLmax_swivel = maxDist_swivel - minDist_swivel;
    % end
    
    %% Find the torques on the shoulder given by the exo during FRONTAL ELEVATION
    Tau_exo_shoulder = zeros(length(Pos.wrSwivel), 3);   Tau_exo_elbow  = zeros(length(Pos.wrSwivel),3);
    
    for j = 1:size(Pos.elbowSwivel,1)    
    Tau_sh = zeros(1,3);       Tau_swivel = zeros(1,3);

        for i = 1:numbsh  

            % [Tau_marionet_elevation, T_elevation(j,i), T_dist_elevation(j,i)] = tauMARIONET3D( Pos.sh,  Pos.gapel(j,:), Actual_Pin_elevation(i,:), ...
            %                                                                                 l0_elevation(i), ...
            %                                                                                 p_el( 7 + Exo.nParamsSh*(i-1)) * Exo.K, ...
            %                                                                                 p_el( 8 + Exo.nParamsSh*(i-1)) * Exo.K, ...
            %                                                                                 l0_recoil_el(i), ...
            %                                                                                 Exo.L_rail, DLmax_elevation(i), ...
            %                                                                                 L0_pre_ext_el(i) );  

            [Tau_marionet_elevation, T_elevation(j,i), T_dist_elevation(j,i), func_tension] = tauMARIONETBUNJEE( Pos.sh,  Pos.gapel(j,:), Actual_Pin_elevation(i,:), l0_elevation(i)); %, ...
                                                                                              
            Tau_sh = Tau_sh + Tau_marionet_elevation(1:3);
            %Tau_sh_elbow = Tau_sh_elbow + Tau_marionet_elevation(4);
            
        end
        
        for i = 1:numbswivel    

            % [Tau_marionet_swivel, T_swivel(j,i), T_dist_swivel(j,i)] = tauMARIONET3D( Pos.sh, endpointsw(j,:), Actual_Pin_swivel(i,:), ...
            %                                                                        l0_swivel(i), ...
            %                                                                        p_sw( 7 + Exo.nParamsSh*(i-1)) * Exo.K, ...
            %                                                                        p_sw( 8 + Exo.nParamsSh*(i-1)) * Exo.K, ...
            %                                                                        l0_recoil_sw(i), ...
            %                                                                        Exo.L_rail, DLmax_swivel(i), ...
            %                                                                        L0_pre_ext_sw(i));

            [Tau_marionet_swivel, T_swivel(j,i), T_dist_swivel(j,i), func_tension] = tauMARIONETBUNJEE( Pos.sh, Pos.gapsw(j,:), Actual_Pin_swivel(i,:), l0_swivel(i)); %, ...
                                                                      
            % Here I evaluate torque also for the elbow
            Tau_swivel = Tau_swivel + Tau_marionet_swivel(1:3);
            %Tau_swivel_elbow = Tau_swivel_elbow + Tau_marionet_swivel(4);
        end
        
        endpoint_wrist = Pos.elbowSwivel(j,:) + (Pos.sh - Pos.elbowSwivel(j,:) * p_wr( 2 ));  % Attachment on the wrist    
        
        [Tau_marionet_elbow, ~, ~, ~] = tauMARIONETBUNJEE( Pos.elbowSwivel(j,:), Pos.wrSwivel(j,:), endpoint_wrist, l0_wrist);                                      
        
        % Store the torques for the shoulder and elbow
        Tau_exo_shoulder(j,:) = Tau_sh + Tau_swivel;   % + Tau_marionet_elbow(1:3);
        Tau_exo_elbow(j,:)    = Tau_marionet_elbow;    %(4) + Tau_swivel_elbow + Tau_sh_elbow;
    end
    
    %% Collect the Tau from the exo
    TauExo.elevationSh = Tau_exo_shoulder;
    TauExo.elevationEl = Tau_exo_elbow; 
    
    Actual_Pin.swivel    = Actual_Pin_swivel;
    Actual_Pin.elevation = Actual_Pin_elevation;
    Actual_Pin.wrist     = endpoint_wrist;
    
        L_springs.swivel = L_swivel;
    L_springs.elevation  = L_elevation;
    
       L0.swivel = l0_swivel;
    L0.elevation = l0_elevation;
   
       Tension.swivel = T_swivel;
    Tension.elevation = T_elevation;
    
    Tension.T_dist_elevation = T_dist_elevation;
       Tension.T_dist_swivel = T_dist_swivel;

end
