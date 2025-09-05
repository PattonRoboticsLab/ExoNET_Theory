function SaveResults(TauExo, TauResidual, TAUsDesired, Actual_Pin, bestCost, RMSE, bestP_3D, Force_vector)
    
% Find all the figures
figures = findall(0, 'Type', 'figure');
[~, idx] = sort([figures.Number]); % Reorder the figures
figures = figures(idx);  

% Count the number of figures
numFigures = length(figures);

% Find a new folder 
folderName = 'risultatioptPB';
if exist(folderName, 'dir')
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    folderName = [folderName '_' timestamp];
end
mkdir(folderName);

% Angles for the visualization
angles = [40, 80, 120, 160, 200, 240, 290];

% Save the figure with different angolation
for i = 1:numFigures
    fig = figures(i); 
    figure(fig); 
    
    if i == 1 || i == 2
        for j = 1:length(angles)
            view(angles(j), 30); % Modify the angle
            filename = fullfile(folderName, sprintf('figure_%d_%d.png', i, angles(j)));
            saveas(gcf, filename); % Save figure
        end

    else
        filename = fullfile(folderName, sprintf('figure_%d.png', i));
        saveas(gcf, filename);
    end
end

% Save results in new folder
    results = { TauExo,   TauResidual,   TAUsDesired,   Actual_Pin,   bestCost,   RMSE,   bestP_3D,   Force_vector, };
resultNames = {'TauExo', 'TauResidual', 'TAUsDesired', 'Actual_Pin', 'bestCost', 'RMSE', 'bestP_3D', 'Force_vector' };

for i = 1:length(results)
    filename = fullfile(folderName, sprintf('%s.mat', resultNames{i}));
    save(filename, resultNames{i});
end

end
