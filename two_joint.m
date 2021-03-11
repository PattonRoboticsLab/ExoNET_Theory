function [torque1,torque2] = two_joint(theta3,r3,r3_pos,R,phi1,l1,l1_pos,phi2,l2,l2_pos,l0,k)

L1=[l1*cosd(phi1), l1*sind(phi1) ,0];%l_pos(2)+
L2=[l2*cosd(phi2), l2*sind(phi2) ,0];%l_pos(2)+
L=L1+L2;
T=R-L;
[F,F_mod,F_vers,T_norm] = force(l0,T,k);
torque1=cross(L,F);
torque2=cross(L2,F);
end

