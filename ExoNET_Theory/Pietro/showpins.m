%% Function to show the pins and the exo
function showpins(Actual_Pin, Exo, Pos, p)

% shoulder_pin = Exo.shoulder; 
% shoulder_pin = [0.186223, -0.0639, 1.36489]; 
% center_back = [0, -0.146, 1.15];
% 
%  shoulder_el = [0.169316, -0.0167571, 1.39712];    shoulder_sw = [0.195279 -0.0285629 1.37362];
%     elbow_el = [0.17231,   0.2128,  1.22811];        elbow_sw = [0.2066 0.1959 1.20793];
%        wrist = [0.19988, 0.413207, 1.0608];   

%% For flexion
    shoulder_pin = [0.17223, -0.0639, 1.36489]; 
    center_back = [0, -0.146, 1.15];

shoulder_el = [0.179316, -0.0137571, 1.40712];   shoulder_sw = [0.201985, 0.218, 1.25257];
   elbow_el = [0.203602, 0.167063, 1.15331];      elbow_sw = [0.243769,0.151475,1.15126];
      wrist = [0.2119, 0.39243,1.20083];   
      
nancy_body;  PlotPin( Exo, Actual_Pin, shoulder_pin, shoulder_el, shoulder_sw, elbow_sw, elbow_el, p, center_back, wrist, Pos);
 %legend(legend6); 