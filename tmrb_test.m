%% update PLOTS
clear all; close all; clc; 
load('EA_latest.mat')

figure()
clf; subplot(1,2,1); drawBody2(Bod.pose,Bod);              % cartoon man, posed
drawExonets(p,Bod.pose);                          % exonet lineSegs
TAUs=exoNetTorques(p,PHIs, 'plotIt');             % solution calc
%TAUs=exoNetTorques(bestP,PHIs);
plotVectField(PHIs,Bod,Pos,TAUsDesired,'r');          % desired again
plotVectField(PHIs,Bod,Pos,TAUs,'b');                 % plot solution 
%fig = gcf();
savefig('current_solution.fig');
% PHIs=PHIsWorkspace; Pos=PosWorkspace;               % add fullWorkspace
% TAUs=exoNetTorques(p,PHIs);                         % field @these 
% plotVectField(PHIs,Bod,Pos,TAUs,'b');               % also plot these
subplot(1,2,2); axis(ax2); subplot(1,2,1); axis(ax1); % zoom frame
title([ProjectName ',  AvgError=' num2str(meanErr)]); % show the goods
