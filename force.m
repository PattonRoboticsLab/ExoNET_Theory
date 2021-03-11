function [F,F_mod,F_vers,T_norm] = force(l0,T,k)
T_norm = norm(T);
if T_norm>l0
    F_vers = T./T_norm;
    F_mod=k*(T_norm-l0);
    F=F_mod*F_vers;
else
    F_mod=0;
    F_vers=[0 0 0];
    F=[0 0 0];
end
end

