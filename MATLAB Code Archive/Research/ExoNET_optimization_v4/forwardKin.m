forwardKin: 
% modified from carellas forwardKin
%
%                        j2
%                        o   q2  `
%                         \     `                
%                         (m2) `                      
%                           \ `               
%                           o j1    
%                          /           
%                        (m1)          
%                        /   q1        
%                    __o___` ` ` ` ` `  
%                    \\\\\\\         
%
%    L    distal-to-segment mass ctr., 
%    q:   list of angles, column 1 is q1, column 2 is q2 +countrclockwise


function [j1,j2]=forwardKin(q)

%% forward kin
for i=1:size(q,1)
  q12 = q(i,1)+q(i,2);
  R_el = [cos(q(i,1))  -sin(q(i,1)); 
          sin(q(i,1))   cos(q(i,1)) ];      % rot matrix
  R_wr = [cos(q12)     -sin(q12); 
          sin(q12)      cos(q12)  ];        % rot matrix
  j1 = sh_pos+(R_el*[L(1); 0]);
  j2= j1+(R_wr*[L(2); 0]);
end
