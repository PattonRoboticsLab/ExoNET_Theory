
function fitForce=tension(x)
% Linear model Poly7 from curve fitting toolbox
% Coefficients (with 95% confidence bounds):
       p1 =       ;
       p2 =      -717.9 ;
       p3 =        2282;
       p4 =       -3589;
       p5 =        2798 ;
       p6 =      -861.8 ;
fitForce= (89.54*x.^5 + p2*x.^4 + p3*x.^3 + p4*x.^2 + p5*x + p6).*(x>1);

end