clc
clear all
close all

%% Stiffness Function without ratios
%  tension = @(L0,L) (L0 < L)*(8.646 - 2.393*L0 + 3.119 * L + .3573*L0^2 + ...
%     1.856*L0*L - 1.898*L^2 - 15.19*L0^3 + 41.97*L0^2*L - 38.93*L0*L^2 + ...
%     13.72*L^3 + 21.03*L0^4 - 77.65*L0^3*L + 104.9*L0^2*L^2 - 62.11*L0*L^3 + ...
%     13.44*L^4 - 7.832*L0^5 + 37.05*L0^4*L - 69.34*L0^3*L^2 + 63.52*L0^2*L^3 - ...
%     29.13*L0*L^4 + 5.317*L^5);

%% Stiffness Function With Ratios
%tension = @(L0,L) (-2.098*(L/L0).^6+2.101*(L/L0).^5+7.546*(L/L0).^4-2.2*(L/L0).^3-5.008*(L/L0).^2+4.423*(L/L0)+9.343).*((L-L0)>0); 


%% Sample Resting/Stretch Values for Testing
Lo = [.15 ]  %, .20, .25, .3, .35, .4, .45, .5];
stch = [-.05, -.02, 0, .001, .01, .02, .03, .04, .05, .06, .07, .08, .09, .1,.11, .12, .13, .14, .15, .16, .17, .18, .19, .20, .21,.22, .23, .24, .25, .26, .27, .28, .29, .30, .31, .32, .35, .36, .37, .38, .40, .5, .6];

stch=-.1:.001:.2

T = stch*NaN;
clf

%% Plot
for i = 1:length(Lo)
    for j = 1:length(stch)
        Tdist = Lo(i) + stch(j);
        T(j) = tension(Lo(i), Tdist);
        hold on
    end
        plot(stch+Lo(i),T, '.-')
end



