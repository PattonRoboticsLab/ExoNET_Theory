%% update PLOTS
clear all; close all; clc; 
load('EA_latest.mat')

f = figure;
ax = axes('Parent',f,'position',[0.13 0.39  0.77 0.54]);
clf; 
drawBody2(Bod.pose,Bod);              % cartoon man, posed
h = drawExonets(p,Bod.pose);   

b = uicontrol('Parent',f,'Style','slider','Position',[81,54,419,23],...
              'value',zeta, 'min',0, 'max',1);
bgcolor = f.Color;
bl1 = uicontrol('Parent',f,'Style','text','Position',[50,54,23,23],...
                'String','0','BackgroundColor',bgcolor);
bl2 = uicontrol('Parent',f,'Style','text','Position',[500,54,23,23],...
                'String','1','BackgroundColor',bgcolor);
bl3 = uicontrol('Parent',f,'Style','text','Position',[240,25,100,23],...
                'String','Damping Ratio','BackgroundColor',bgcolor);

b.Callback = @(es,ed) updateSystem(h,drawExonets(p,Bod.pose)); 