function [F,F_mod,F_vers,T_norm,E_pot] = force(l0,T,k)
T_norm = norm(T);
if T_norm>l0
    F_vers = T./T_norm;
    F_mod=k*(T_norm-l0);
    F=F_mod*F_vers;
    E_pot=0.5*k*(T_norm-l0)^2;
else
    F_mod=0;
    F_vers=[0 0 0];
    F=[0 0 0];
end
end

