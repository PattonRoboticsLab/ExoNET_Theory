% load('e.mat');
% 
% figure
% subplot(1,2,1)
% plot1DDistribution(e(:,1),'b')
% title('Hip Torque Error')
% 
% subplot(1,2,2)
% plot1DDistribution(e(:,2),'g')
% title('Knee Torque Error')

% axis equal uses the same length for the data units along each axis
close all

%% FIGURE ON THE NEW PAPER
figure

load('err_gc_1e2j.mat')
plot2DDistribution(a(:,1),a(:,2),'m')
%plot2DDistribution(err_DA_1e2j(:,1),err_DA_1e2j(:,2),'m')
hold on


load('err_gc_1e.mat')
plot2DDistribution(b(:,1),b(:,2),'g')
hold on


load('err_gc_5e.mat')
plot2DDistribution(c(:,1),c(:,2),'b')

% title('Gravity Compensation')
% labels = {'No 2-Joint Elements ExoNET','Single Element ExoNET','Multi-Joint Multi-Element ExoNET'};
%legend(labels,'Location','Southeast')
set(gca,'FontSize',20)
