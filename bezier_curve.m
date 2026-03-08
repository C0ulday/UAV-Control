function [B, Bd, Bdd] = bezier_curve(t, P, n)
    % t: vecteur de temps normalisé [0, 1]
    % P: matrice des points de contrôle (n+1 x 2 ou 3)
    % n: degré de la courbe de Bézier
    % Retourne:
    % B: positions sur la courbe
    % Bd: dérivée première (vitesse)
    % Bdd: dérivée seconde (accélération)
    
    % Initialisation des sorties
    num_points = length(t);
    dim = size(P, 2); % 2D ou 3D
    B   = zeros(num_points, dim);
    Bd  = zeros(num_points, dim);
    Bdd = zeros(num_points, dim);
    
    % Calcul pour chaque point de contrôle
    for i = 0:n
        % Coefficients binomiaux et termes de Bernstein
        binom = nchoosek(n, i);
        bernstein = binom * (1 - t).^(n - i) .* t.^i;
        
        % Contribution à la position (ajout colonne)
        B = B + bernstein' .* P(i+1, :);
        
        % Contribution à la dérivée première (si n >= 1)
        if n >= 1 && i < n
            delta = P(i+2, :) - P(i+1, :);
            term = n * binom * (1 - t).^(n-1 - i) .* t.^i;
            Bd = Bd + term' .* delta; % Transposition critique
        end
        
        % Contribution à la dérivée seconde (si n >= 2)
        if n >= 2 && i < n-1
            delta2 = P(i+3, :) - 2*P(i+2, :) + P(i+1, :);
            term = n*(n-1) * binom * (1 - t).^(n-2 - i) .* t.^i;
            Bdd = Bdd + term' .* delta2; % Transposition critique
        end
    end
end