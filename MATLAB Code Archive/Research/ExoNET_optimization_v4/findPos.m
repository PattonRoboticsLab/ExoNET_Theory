% MATLAB function Pos=findPos(PHIs,Bod) 
%
function Pos=findPos(PHIs,Bod) 

Pos.sh    = zeros(size(PHIs));    % zeros for this one shoulder positons
phi12     = PHIs(:,1)+PHIs(:,2);  % sum phi's
Pos.R1    = [Bod.R(1)*cos(PHIs(:,1)), Bod.R(1)*sin(PHIs(:,1))]; %humerousCM
Pos.el    = [Bod.L(1)*cos(PHIs(:,1)), Bod.L(1)*sin(PHIs(:,1))]; % elbow pos
Pos.el2R2 = [Bod.R(2)*cos(PHIs(:,2)),Bod.R(2)*sin(PHIs(:,2))];   % elbow2forearm CM
Pos.R2    = [Pos.el(:,1)+Pos.el2R2(1),Pos.el(:,2)+Pos.el2R2(2)];%arm&handCM
Pos.wr    = [Pos.el(:,1)+Bod.L(2)*cos(PHIs(:,1)+PHIs(:,2)), ...
             Pos.el(:,2)+Bod.L(2)*sin(PHIs(:,1)+PHIs(:,2))]; % wrist pos