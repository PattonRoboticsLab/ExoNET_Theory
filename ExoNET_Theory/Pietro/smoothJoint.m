function smoothJoint(signal, frameList, fs, jointLabel)
% smoothJoint - Confronta diversi smoothing SGOLAY su un segnale articolare
%
% INPUT:
%   signal     : vettore Nx1 (una coordinata, es: shoulder(:,1))
%   frameList  : array con valori di frameLength dispari, es: [101 201 301]
%   fs         : frequenza di campionamento (es: 30)
%   jointLabel : stringa per etichettare il plot (es: 'Shoulder X')

% Tempo
N = length(signal);
t = linspace(0, N/fs, N);

% Colori per i plot
colors = lines(length(frameList));

% Plot
figure; hold on;
plot(t, signal, 'k--', 'DisplayName', 'Raw');

for i = 1:length(frameList)
    frame = frameList(i);
    if mod(frame,2) == 0
        warning('FrameLength %d is not odd, skipping.', frame);
        continue;
    end
    smoothed = sgolayfilt(signal, 3, frame);
    plot(t, smoothed, 'Color', colors(i,:), ...
        'DisplayName', sprintf('SGOLAY frame %d', frame));
end

xlabel('Time (s)');
ylabel('Position');
title(sprintf('Savitzky-Golay smoothing - %s', jointLabel));
legend();
grid on;
end
