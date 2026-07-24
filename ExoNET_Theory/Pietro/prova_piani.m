clear all; clc; close all;
% Definisci il centro dove vuoi che passino i pian
% Crea una figura 3D
figure;

% Definisci il centro
centro = [0, 0, 0];

% Prima fai il plot 3D di base (se hai altri dati)
hold on;
% Trasparenza alta (più trasparente)
aggiungi_piani_rotazione(centro, 2.0, 3.0, 4, [0.95 0.95 0.95], 0.1);

axis equal;
view(3);
hold off;