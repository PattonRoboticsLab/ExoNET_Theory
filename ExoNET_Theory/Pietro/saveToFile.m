% Funzione di output per salvare i risultati
function stop = saveToFile(results, state)
    stop = false;
    if strcmp(state, 'iteration')
        save('BayesoptResults.mat', 'results');
    end
end