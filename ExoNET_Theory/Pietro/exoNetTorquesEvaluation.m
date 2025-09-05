%% Function to evaluate the torque of the exo in the space 
function [TauExo, Actual_Pin, L_springs, L0, Tension, L0_recoil ] = exoNetTorquesEvaluation (Body, Bod, Exo, p_init, armSide)

% Filter the position
fs = 30;
[b, a] = butter( 7, 3 / ( fs / 2 ) );  % Create the filter

% Extract positions, filter them and remove start and end points
validRows = ~any(isnan([Body.Shoulder, Body.Elbow, Body.Wrist,Body.Neck]), 2);
shoulder_raw  = Body.Shoulder(validRows,:);
elbow_raw     = Body.Elbow(validRows,:);
wrist_raw     = Body.Wrist(validRows,:);
neck_raw      = Body.Neck(validRows,:); % The other shoulder helps to create the exo around the shoulder properly

shoulder     = movmean( filtfilt( b, a, shoulder_raw),     13, 'Endpoints', 'shrink');
elbow        = movmean( filtfilt( b, a, elbow_raw),        13, 'Endpoints', 'shrink');
wrist        = movmean( filtfilt( b, a, wrist_raw),        13, 'Endpoints', 'shrink');
neck         = movmean( filtfilt( b, a, neck_raw), 13, 'Endpoints', 'shrink');

shoulder     = shoulder( 90:end-90, :);
elbow        = elbow( 90:end-90, :);
wrist        = wrist( 90:end-90, :);
neck         = neck( 90:end-90, :);

nFrames = length(shoulder);

%% Evaluates numbers of parameters and params per joint
nparams = Exo.nParamsSh * round( Exo.numbconstraints(2) ); 
    
%% Fix P vector
       p_wr = p_init( end-2 : end );
  p_el(:,1) = p_init( 1 : nparams+1 );
  p_sw(:,1) = p_init( nparams+2 : nparams*2+2 );

%% Set the pins, L0 and endpoint
       numbsh = round( p_el(1) );                         numbswivel = round( p_sw(1) );    
 l0_elevation = zeros(1,numbsh);                           l0_swivel = zeros(1,numbswivel);
 l0_recoil_el = zeros(1,numbsh);                        l0_recoil_sw = zeros(1,numbswivel);
L0_pre_ext_el = zeros(1,numbsh);                       L0_pre_ext_sw = zeros(1,numbswivel);
  L_elevation = zeros(nFrames,numbsh);                      L_swivel = zeros(nFrames,numbswivel );
  T_elevation = zeros(nFrames,numbsh);                      T_swivel = zeros(nFrames,numbswivel );

Tau_exo_shoulder = zeros(nFrames, 3);
Tau_exo_elbow    = zeros(nFrames, 3);

%% Select side and correct some values
if lower(armSide) == 'l'
    armLabel  = 'Left';
else
    armLabel  = 'Right';
end

%% Compute static L0 and recoil parameters
for i = 1:numbsh
    l0_elevation(i) = ( Bod.L(1) + p_el( 2 + Exo.nParamsSh*(i-1) ) ) / Exo.stretch_ratio_limit + ...
                     (( Bod.L(1) + p_el( 2 + Exo.nParamsSh*(i-1) ) ) / Exo.stretch_ratio_limit) * p_el( 5 + Exo.nParamsSh*(i-1));
    
    l0_recoil_el(i) = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_el( 9 + Exo.nParamsSh*(i-1));
    
    L0_pre_ext_el(i) = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_el( 10 + Exo.nParamsSh*(i-1));
end

for i = 1:numbswivel
    l0_swivel(i) =  ( Bod.L(1) + p_sw( 2 + Exo.nParamsSh*(i-1) ) ) / Exo.stretch_ratio_limit + ...
                   (( Bod.L(1) + p_sw( 2 + Exo.nParamsSh*(i-1) ) ) / Exo.stretch_ratio_limit) * p_sw( 5 + Exo.nParamsSh*(i-1)); 
    
    l0_recoil_sw(i) = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_sw( 9 + Exo.nParamsSh*(i-1));
    
    L0_pre_ext_sw(i) = Exo.L_rail / Exo.stretch_ratio_limit + (Exo.L_rail / Exo.stretch_ratio_limit) * p_sw( 10 + Exo.nParamsSh*(i-1));
end

    l0_wrist = ( Bod.L(2) + Bod.L(1) ) / Exo.stretch_ratio_limit + ( Bod.L(2) + Bod.L(1) ) * p_wr( 1 ); % Divided by four to see if it's working

%% Main loop for dynamic pin evaluation and torque computation
for j = 1:nFrames
    Tau_sh = zeros(1,3);  Tau_swivel = zeros(1,3);
    
    dir_vec = shoulder(j,:) - neck(j,:);
    dir_vec = dir_vec / norm(dir_vec);
    R = [dir_vec; 0 -1 0; 0 0 1];

    % Evaluate elevation pins and torques
    for i = 1:numbsh
        
         local_vec = [ ...
            p_el(4 + Exo.nParamsSh*(i-1)), ...
            p_el(2 + Exo.nParamsSh*(i-1)) * cos(p_el(3 + Exo.nParamsSh*(i-1))), ...
            p_el(2 + Exo.nParamsSh*(i-1)) * sin(p_el(3 + Exo.nParamsSh*(i-1)))];
        
         Actual_Pin_elevation = shoulder(j,:) + local_vec * R;

        % 
        endpoint = elbow(j,:) + ( shoulder(j,:) - elbow(j,:) ) * p_el( 6 + Exo.nParamsSh*(i-1)) ; % Attachment on the elbow
        L_elevation(j,i) = norm(endpoint - Actual_Pin_elevation);    
  
        [Tau_marionet_elevation, T_elevation(j,i), T_dist_elevation(j,i)] = tauMARIONET3D( ...
                                                        shoulder(j,:), endpoint, Actual_Pin_elevation, ...
                                                        l0_elevation(i), ...
                                                        p_el(7 + Exo.nParamsSh*(i-1)) * Exo.K, ...
                                                        p_el(8 + Exo.nParamsSh*(i-1)) * Exo.K, ...
                                                        l0_recoil_el(i), ...
                                                        Exo.L_rail, max(L_elevation(:,i)) - min(L_elevation(:,i)), ...
                                                        L0_pre_ext_el(i) );

        Tau_sh = Tau_sh + Tau_marionet_elevation;
    end

    % Evaluate swivel pins and torques
    for i = 1:numbswivel
         local_vec = [ ...
            p_sw(4 + Exo.nParamsSh*(i-1)), ...
            p_sw(2 + Exo.nParamsSh*(i-1)) * cos(p_sw(3 + Exo.nParamsSh*(i-1))), ...
            p_sw(2 + Exo.nParamsSh*(i-1)) * sin(p_sw(3 + Exo.nParamsSh*(i-1)))];
        
         Actual_Pin_swivel = shoulder(j,:) + local_vec * R;
        
        endpointsw = elbow(j,:) + (shoulder(j,:) - elbow(j,:)) * p_sw(6 + Exo.nParamsSh*(i-1));
        L_swivel(j,i) = norm(endpointsw - Actual_Pin_swivel);

        [Tau_marionet_swivel, T_swivel(j,i), T_dist_swivel(j,i)] = tauMARIONET3D( ...
                                                        shoulder(j,:), endpointsw, Actual_Pin_swivel, ...
                                                        l0_swivel(i), ...
                                                        p_sw(7 + Exo.nParamsSh*(i-1)) * Exo.K, ...
                                                        p_sw(8 + Exo.nParamsSh*(i-1)) * Exo.K, ...
                                                        l0_recoil_sw(i), ...
                                                        Exo.L_rail, max(L_swivel(:,i)) - min(L_swivel(:,i)), ...
                                                        L0_pre_ext_sw(i) );

        Tau_swivel = Tau_swivel + Tau_marionet_swivel;
    end

    % Wrist torque
    endpoint_wrist = elbow(j,:) + ( shoulder(j,:) - elbow(j,:) ) * p_wr(2);
    [Tau_marionet_elbow, ~, ~] = tauMARIONET_wr( elbow(j,:), wrist(j,:), endpoint_wrist, l0_wrist, p_wr(3) * Exo.K );
    
    Tau_elbow = Tau_marionet_elbow;
    
    % Store torques
    Tau_exo_shoulder(j,:) = Tau_sh + Tau_swivel;
    Tau_exo_elbow(j,:)    = Tau_elbow;
    
end

%% Collect results
TauExo.elevationSh = Tau_exo_shoulder;
TauExo.elevationEl = Tau_exo_elbow;

Actual_Pin.swivel    = Actual_Pin_swivel;
Actual_Pin.elevation = Actual_Pin_elevation;
Actual_Pin.wrist     = endpoint_wrist;

L_springs.swivel     = L_swivel;
L_springs.elevation  = L_elevation;

L0.swivel = l0_swivel;
L0.elevation = l0_elevation;

L0_recoil.swivel = l0_recoil_sw;
L0_recoil.elevation = l0_recoil_el;

Tension.swivel = T_swivel;
Tension.elevation = T_elevation;

Tension.T_dist_elevation = T_dist_elevation;
Tension.T_dist_swivel = T_dist_swivel;


end
