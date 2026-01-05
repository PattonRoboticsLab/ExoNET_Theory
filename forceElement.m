


%% construct a composite function from force and stretch of 
function F=forceElement(L0,L)

normL=L./L0;
F=NaN*ones(size(L));
polyCoef=[ -0.644671   % organge Theraband, see regressForceElement.m
           10.025959 
          -65.087812 
          227.630569 
         -460.710799 
          537.211441 
         -337.115591 
           99.723534 
          -10.474202 
            0.138370];

for i=1:length(L)
  if      normL(i)<=1,  F(i)=0;
  elseif  normL(i)>3,   F(i)=-3.8+5.4*normL(i); % from trial&error, extrap  
  else                  F(i)= polyval(polyCoef,normL(i));
  end % END if
end % END for

end % END function


