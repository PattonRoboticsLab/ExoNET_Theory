function [torque] = one_joint(theta,r,r_pos,R,phi,l,l_pos,l0,k)

L=[l*cosd(phi), l*sind(phi) ,0];%l_pos(2)+
T=R-L;
[F,F_mod,F_vers,T_norm] = force(l0,T,k);
torque=cross(L,F);
end

