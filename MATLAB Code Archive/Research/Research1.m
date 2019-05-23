clc
clear all

%1 Stack MARIONET
rng default
n=1; %Number of Stacks
L = .34; %Length of Arm
phi = linspace(0,3); %Phi Angles
torque = sin(phi)+cos(phi); %Desired Torque Distribution
lb = []; %Lower Bounds of Optimized Values
ub = []; %Upper Bounds of Optimized Values
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi))); %Function defining the moment arm of the force
%p(1) = angle p(2) = radius
x0 = zeros(1,2*n); %Initial Guesses for Optimized P values
p = lsqcurvefit(fun,x0,phi,torque,lb,ub); %Least Squares Fit Optimizations
figure(1) %Plotting Phi, Torque relationship with Optimized Values
plot(phi,torque,'ko',phi,fun(p,phi),'r')
residual1 = torque-fun(p,phi);


hold on
%2 Stack MARIONET
n=2;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
%plot(phi,torque,'ko',phi,fun(p,phi),'g-')
residual2 = torque-fun(p,phi);


%3 Stack MARIONET
n=3;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))+(L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
plot(phi,torque,'ko',phi,fun(p,phi),'g')
residual3 = torque-fun(p,phi);

%4 Stack MARIONET
n=4;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))+(L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)))+(L*p(8)*sin(p(7)-phi))./sqrt(p(8)^2+L^2-(2*p(8)*L*cos(p(7)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
%plot(phi,torque,'ko',phi,fun(p,phi),'p-')
residual4 = torque-fun(p,phi);

%5 Stack MARIONET
n=5;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))+(L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)))+(L*p(8)*sin(p(7)-phi))./sqrt(p(8)^2+L^2-(2*p(8)*L*cos(p(7)-phi)))+(L*p(10)*sin(p(9)-phi))./sqrt(p(10)^2+L^2-(2*p(10)*L*cos(p(9)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
plot(phi,torque,'ko',phi,fun(p,phi),'b')
residual5 = torque-fun(p,phi);

%6 Stack MARIONET
n=6;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))+(L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)))+(L*p(8)*sin(p(7)-phi))./sqrt(p(8)^2+L^2-(2*p(8)*L*cos(p(7)-phi)))+(L*p(10)*sin(p(9)-phi))./sqrt(p(10)^2+L^2-(2*p(10)*L*cos(p(9)-phi)))+(L*p(12)*sin(p(11)-phi))./sqrt(p(12)^2+L^2-(2*p(12)*L*cos(p(11)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
%plot(phi,torque,'ko',phi,fun(p,phi),'p-')
residual6 = torque-fun(p,phi);

%7 Stack MARIONET
n=7;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))+(L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)))+(L*p(8)*sin(p(7)-phi))./sqrt(p(8)^2+L^2-(2*p(8)*L*cos(p(7)-phi)))+(L*p(10)*sin(p(9)-phi))./sqrt(p(10)^2+L^2-(2*p(10)*L*cos(p(9)-phi)))+(L*p(12)*sin(p(11)-phi))./sqrt(p(12)^2+L^2-(2*p(12)*L*cos(p(11)-phi)))+(L*p(14)*sin(p(13)-phi))./sqrt(p(14)^2+L^2-(2*p(14)*L*cos(p(13)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
plot(phi,torque,'ko',phi,fun(p,phi),'m')
residual7 = torque-fun(p,phi);

%8 Stack MARIONET
n=8;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))+(L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)))+(L*p(8)*sin(p(7)-phi))./sqrt(p(8)^2+L^2-(2*p(8)*L*cos(p(7)-phi)))+(L*p(10)*sin(p(9)-phi))./sqrt(p(10)^2+L^2-(2*p(10)*L*cos(p(9)-phi)))+(L*p(12)*sin(p(11)-phi))./sqrt(p(12)^2+L^2-(2*p(12)*L*cos(p(11)-phi)))+(L*p(14)*sin(p(13)-phi))./sqrt(p(14)^2+L^2-(2*p(14)*L*cos(p(13)-phi)))+(L*p(16)*sin(p(15)-phi))./sqrt(p(16)^2+L^2-(2*p(16)*L*cos(p(15)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
%plot(phi,torque,'ko',phi,fun(p,phi),'p-')
residual8 = torque-fun(p,phi);

%9 Stack MARIONET
n=9;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))+(L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)))+(L*p(8)*sin(p(7)-phi))./sqrt(p(8)^2+L^2-(2*p(8)*L*cos(p(7)-phi)))+(L*p(10)*sin(p(9)-phi))./sqrt(p(10)^2+L^2-(2*p(10)*L*cos(p(9)-phi)))+(L*p(12)*sin(p(11)-phi))./sqrt(p(12)^2+L^2-(2*p(12)*L*cos(p(11)-phi)))+(L*p(14)*sin(p(13)-phi))./sqrt(p(14)^2+L^2-(2*p(14)*L*cos(p(13)-phi)))+(L*p(16)*sin(p(15)-phi))./sqrt(p(16)^2+L^2-(2*p(16)*L*cos(p(15)-phi)))+(L*p(18)*sin(p(17)-phi))./sqrt(p(18)^2+L^2-(2*p(18)*L*cos(p(17)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
%plot(phi,torque,'ko',phi,fun(p,phi),'p-')
residual9 = torque-fun(p,phi);

%10 Stack MARIONET
n=10;
lb = [];
ub = [];
fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))+(L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))+(L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)))+(L*p(8)*sin(p(7)-phi))./sqrt(p(8)^2+L^2-(2*p(8)*L*cos(p(7)-phi)))+(L*p(10)*sin(p(9)-phi))./sqrt(p(10)^2+L^2-(2*p(10)*L*cos(p(9)-phi)))+(L*p(12)*sin(p(11)-phi))./sqrt(p(12)^2+L^2-(2*p(12)*L*cos(p(11)-phi)))+(L*p(14)*sin(p(13)-phi))./sqrt(p(14)^2+L^2-(2*p(14)*L*cos(p(13)-phi)))+(L*p(16)*sin(p(15)-phi))./sqrt(p(16)^2+L^2-(2*p(16)*L*cos(p(15)-phi)))+(L*p(18)*sin(p(17)-phi))./sqrt(p(18)^2+L^2-(2*p(18)*L*cos(p(17)-phi)))+(L*p(20)*sin(p(19)-phi))./sqrt(p(20)^2+L^2-(2*p(20)*L*cos(p(19)-phi)));
x0 = zeros(1,2*n);
p = lsqcurvefit(fun,x0,phi,torque,lb,ub);
residual10 = torque-fun(p,phi);

plot(phi,torque,'ko',phi,fun(p,phi),'b')
legend('Desired Torque','N=1','N=3','N=5','N=7','N=10')
title('Desired Torque, MARIONET Torque and Single Components')
xlabel('{\phi} (rad)')
ylabel('Torque (Nm)')

%Error plot
figure(2)
Error = [mean(residual1) mean(residual2) mean(residual3) mean(residual4) mean(residual5) mean(residual6) mean(residual7) mean(residual8) mean(residual9) mean(residual10)];
N = 1:1:10;
plot(N,Error)
title('Average Error vs Number of Stacks')
xlabel('Number of Stacks')
ylabel('Average Error (Nm)')

%Bistable Energy Profile
phi = linspace(0,3);
Energy = .5*L^2+p(2)^2-2*L*p(2).*cos(p(1)-phi); %Energy Represented by Equation (1/2)kx^2
figure(3)
plot(phi,Energy) %Plot
title('Bistable Energy Profile')
xlabel('{\phi} (rad)')
ylabel('Energy (J)')


%Functions Below were used for the manual Stack Addition
%fun = @(p,phi) (L*p(2)*sin(p(1)-phi))./sqrt(p(2)^2+L^2-(2*p(2)*L*cos(p(1)-phi)))
%fun2 = (L*p(4)*sin(p(3)-phi))./sqrt(p(4)^2+L^2-(2*p(4)*L*cos(p(3)-phi)))
%fun3 = (L*p(6)*sin(p(5)-phi))./sqrt(p(6)^2+L^2-(2*p(6)*L*cos(p(5)-phi)))
%fun4 = (L*p(8)*sin(p(7)-phi))./sqrt(p(8)^2+L^2-(2*p(8)*L*cos(p(7)-phi)))
%fun5 = (L*p(10)*sin(p(9)-phi))./sqrt(p(10)^2+L^2-(2*p(10)*L*cos(p(9)-phi)))
%fun6 = (L*p(12)*sin(p(11)-phi))./sqrt(p(12)^2+L^2-(2*p(12)*L*cos(p(11)-phi)))
%fun7 = (L*p(14)*sin(p(13)-phi))./sqrt(p(14)^2+L^2-(2*p(14)*L*cos(p(13)-phi)))
%fun8 = (L*p(16)*sin(p(15)-phi))./sqrt(p(16)^2+L^2-(2*p(16)*L*cos(p(15)-phi)))
%fun9 = (L*p(18)*sin(p(17)-phi))./sqrt(p(18)^2+L^2-(2*p(18)*L*cos(p(17)-phi)))
%fun10 = (L*p(20)*sin(p(19)-phi))./sqrt(p(20)^2+L^2-(2*p(20)*L*cos(p(19)-phi)))
%RESTING LENGTH, 

