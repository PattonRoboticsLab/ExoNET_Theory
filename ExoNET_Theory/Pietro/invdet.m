A=[(53*cos((4*pi)/9))/100, cos((4*pi)/9)/4, 0; (53*sin((4*pi)/9))/100, sin((4*pi)/9)/4,  0;  0, 0, -(53*cos((4*pi)/9))/100]
det(A)
inv(A)
syms phi1 phi2 phi3 L1 L2

J=[L1,L2]*[sin(phi1), sin(phi3)*cos(phi1) ; sin(phi1+phi2), sin(phi3)*cos(phi1+phi2)];
 
Jacobians=jacobian( J , [phi1 phi2] );
Jacobians=subs(Jacobians,{phi1,phi2,phi3,L1,L2},{PHIs(2,1),PHIs(2,2),PHIs(2,3),Bod.L(1),Bod.L(2)})
det(Jacobians)
invJacob=inv(Jacobians')
