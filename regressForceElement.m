% Function for 
% 

function regressForceElement()
plotIt='Yes, Plot It';

%% Data
orange=[...
0	0
.01 0
.05 0
.1 0
.2 0
.3 0
.4 0
.5 0
.6 0
.7 0
.8 0
.9 0
.93 0
.95 0
.97 0
1	0
1.041666667	1
1.083333333	1.4
1.125	2.2
1.166666667	2.2
1.208333333	2.8
1.25	3
1.291666667	3.6
1.333333333	4
1.375	4.2
1.416666667	4.6
1.458333333	4.8
1.5	5
1.541666667	5.2
1.583333333	5.6
1.625	5.8
1.666666667	6
1.708333333	6.2
1.75	6.4
1.791666667	6.6
1.833333333	6.8
1.875	6.8
1.916666667	7.2
1.958333333	7.4
2	7.6
2.041666667	7.6
2.083333333	7.8
2.125	8.2
2.166666667	8.4
2.208333333	8.6
2.25	8.6
2.291666667	8.8
2.333333333	9
2.375	9.2
2.416666667	9.4
2.458333333	9.8
2.5	9.8
2.541666667	10
2.583333333	10.4
2.625	10.4
2.666666667	10.8
2.708333333	10.8
2.75	11.2
2.791666667	11.4
2.833333333	11.6
2.875	11.8
2.916666667	12
3	12.4 
3.1 13 % made up to alow better fit
3.2 13.6 % made up to alow better fit
3.3 14 % made up to alow better fit
]; 
% 2.958333333	12.4  % removed

normL=orange(:,1);
Force=orange(:,2); 
mat2txt('orangeTheraBand.txd',char('Force-Length orange theraBand tube',...
  ['L/L0' char(9) 'F']) ,[normL Force]);

%% polynomial fit
polyCoef = polyfit(normL,Force,9)';
fprintf('\npolyCoef=')
for i=1:length(polyCoef), fprintf('\n %9f ',polyCoef(i)); end
x=-.01:.01:3.28;  % new L/L0
F1=polyval(polyCoef,x); 
L=-.5:.05:4; L0=1; % for checking the extrapolation
F=compositeForceElement(polyCoef,L,L0);

%% Plot
if plotIt,
  fprintf('Plotting force element..');
  clf; plot(normL,Force,'.-', 'markersize',16);
  jimPlot; hold on; axis on;
  plot(x,F1,'r','linewidth',2);
  ylabel('Force');
  xlabel('normalized Length');
  plot(L,F,'g','linewidth',2);
  fprintf('done. \n');
end % end if plotIt

end % END function

%% construct a composite function with regression in the middle
function F=compositeForceElement(polyCoef,L,L0)

normL=L./L0;
F=NaN*ones(size(L));
intercept=-3.8; % from trial&error
slope=5.4; % from trial& error

for i=1:length(L)
  if      normL(i)<=1, F(i)=0;                         % slack 
  elseif  normL(i)>3,  F(i)=intercept+slope*normL(i);  % extrap beyond data 
  else                 F(i)=polyval(polyCoef,normL(i));% from regression
  end % END if
end % END for

end % END function


