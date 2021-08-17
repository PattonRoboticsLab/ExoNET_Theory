%______*** MATLAB "M" file (jim Patton) ***_______
% Inverse kinematics of two-joint planar arm, (such as the human arm)
% SYNTAX:  q=invrskin(p,L)
% INPUTS:  p    list of 2-d positons, colums are x and y.
%
%                     p  o   q2  `
%                         \     `                
%                         L(2) `                      
%                           \ `               
%                           o     
%                          /           
%                        L(1)          
%                        /   q1        
%                    __o___` ` ` ` ` `  
%                    \\\\\\\         
%
%    L is segment lengths 
%    q is cooresponding list of relative angles, positive countrclockwise 
%
% REVISIONS: 1/28/99 Adapeted by patton from conditt's INVRSKIN.M
%            2/6/2019 (patton) modified from that  
%~~~~~~~~~~~~~~~~~~~~~~~~~~  BEGIN ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

function q=inverseKin(p,L)
p=p';
q(2,:)=acos((p(1,:).^2+p(2,:).^2-L(1)^2-L(2)^2)./(2*L(1)*L(2)));
q(1,:)=atan2(p(2,:),p(1,:))-atan2((L(2).*sin(q(2,:))),(L(1)+(L(2).*cos(q(2,:)))));
q=q';


