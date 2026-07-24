function aggiungi_piani_rotazione(centro, larghezza, altezza, num_piani, colore, alpha)
    % AGGIUNGI_PIANI_ROTAZIONE Aggiunge piani rettangolari di rotazione rispetto all'asse Z
    %
    % Parametri:
    %   centro    - Coordinate del centro [x, y, z]
    %   larghezza - Larghezza dei piani rettangolari (estensione radiale dal centro)
    %   altezza   - Altezza dei piani lungo l'asse Z
    %   num_piani - Numero di piani da disegnare (default: 4)
    %   colore    - Colore dei piani (default: [0.8 0.8 0.8] grigio chiaro)
    %   alpha     - Trasparenza dei piani (default: 0.3)
    
    % Valori di default
    if nargin < 4
        num_piani = 4;
    end
    if nargin < 5
        colore = [0.95 0.95 0.95]; % grigio quasi bianco
    end
    if nargin < 6
        alpha = 0.5;
    end
    
    % Mantieni il plot corrente
    wasHeld = ishold;
    hold on;
    
    % Angoli da 0° a 90° in senso antiorario (visto dall'alto sull'asse Z)
    angoli = linspace(0, pi/2, num_piani);
    
    for i = 1:length(angoli)
        angolo = angoli(i);
        
        % Crea una griglia per il piano verticale
        % Il piano è verticale (parallelo all'asse Z) e ruota attorno ad esso
        r_vals = linspace(0, larghezza, 20);  % distanza radiale dal centro
        z_vals = linspace(-altezza/2, altezza/2, 20);  % altezza lungo Z
        [R, Z_grid] = meshgrid(r_vals, z_vals);
        
        % Converti coordinate polari in cartesiane per il piano ruotato
        X = centro(1) + R * cos(angolo);
        Y = centro(2) + R * sin(angolo);
        Z = centro(3) + Z_grid;
        
        % Disegna il piano rettangolare usando surf SENZA griglie
        % Crea una matrice di colore uniforme per evitare interferenze con colormap
        C = ones(size(Z));  % matrice di 1 per colore uniforme
        h = surf(X, Y, Z, C, 'FaceColor', colore, 'FaceAlpha', alpha, ...
                 'EdgeColor', 'none', 'CDataMapping', 'direct');
        
        % Applica illuminazione SOLO a questo oggetto
        set(h, 'FaceLighting', 'gouraud');
    end
    
    % Ripristina lo stato hold precedente
    if ~wasHeld
        hold off;
    end
    
end