
function fitForce=tension(x,Tdist)
% Linear model Poly7 from curve fitting toolbox
% Coefficients (with 95% confidence bounds):
 

       p1 =       747.7;%  (580.2, 915.3)
       p2 =       -8211 ;% (-9954, -6468)
       p3 =   3.838e+04;%  (3.066e+04, 4.609e+04)
       p4 =  -9.892e+04 ;% (-1.177e+05, -8.012e+04)
       p5 =   1.519e+05  ;%(1.246e+05, 1.791e+05)
       p6 =  -1.388e+05  ;%(-1.624e+05, -1.153e+05)
       p7 =   7.002e+04  ;%(5.883e+04, 8.122e+04)
       p8 =  -1.503e+04  ;%(-1.729e+04, -1.276e+04)
       x = Tdist/L0;
       
       fitForce = p1*x^7 + p2*x^6 + p3*x^5 + p4*x^4 + p5*x^3 + p6*x^2 + p7*x + p8;


end