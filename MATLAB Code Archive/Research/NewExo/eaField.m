% eaField: caluate a special force field for horizontal reach
% SYNTAX: tau=eaField(Bod);                     % set torques2cancelGrav
% ~~~ BEGIN ~~~

function [TAUs,PHIs,Pos]=eaField(Bod)                    % set torques2cancelGrav

f=10;

x=[  .2       0
     .3       0
     .4       0
     .5       0
     .2      -.05
     .3      -.05
     .4      -.05
     .5      -.05
     .2      -.1  
     .3      -.1
     .4      -.1
     .5      -.1
     .2      -.15  
     .3      -.15
     .4      -.15
     .5      -.15
     .2      -.2  
     .3      -.2
     .4      -.2
     .5      -.2 ];
plot(x(:,1),x(:,2),'.','color',.6*[1 1 1]); % plot positions grey

F=  [ 0       0
      0       0
      0       0
      0       0
      0       f
      0       f
      0       f
      0       f
      0       0  
      0       0
      0       0
      0       0
      0      -f  
      0      -f
      0      -f
      0      -f
      0       0  
      0       0
      0       0
      0       0 ];

PHIs=inverseKin(x,Bod.L); % 
Pos=findPos(PHIs,Bod);   % positions assoc w/ these angle combinations

for i=1:size(x,1), TAUs(i,:)=((jacobian(PHIs(i,:),Bod.L)')*F(i,:)'); end; % tau=JT*F



