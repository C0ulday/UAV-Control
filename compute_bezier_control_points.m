function P = compute_bezier_control_points(xi, ui, xf, uf, param, n)
    
    % Points de contrôle initiaux et finaux
    P = zeros(n+1, 2);
    P(1,:) = [xi(1), xi(2)]; % Point de départ
    P(end,:) = [xf(1), xf(2)]; % Point d'arrivée
    
    % Calcul des dérivées initiales et finales
    zd1_0 = ui(1) * cos(xi(3)); % Vitesse initiale en x
    zd2_0 = ui(1) * sin(xi(3)); % Vitesse initiale en y
    zdd1_0 = -sin(xi(3)) * param.g * tan(ui(2)); % Accélération initiale en x
    zdd2_0 = cos(xi(3)) * param.g * tan(ui(2)); % Accélération initiale en y
    
    zd1_f = uf(1) * cos(xf(3)); % Vitesse finale en x
    zd2_f = uf(1) * sin(xf(3)); % Vitesse finale en y
    zdd1_f = -sin(xf(3)) * param.g * tan(uf(2)); % Accélération finale en x
    zdd2_f = cos(xf(3)) * param.g * tan(uf(2)); % Accélération finale en y
    
    % Calcul des points de contrôle intermédiaires
    P(2,:) = P(1,:) + [zd1_0 / n, zd2_0 / n]; % Point de contrôle pour la vitesse initiale
    P(end-1,:) = P(end,:) - [zd1_f / n, zd2_f / n]; % Point de contrôle pour la vitesse finale
    
    % Points de contrôle pour l'accélération initiale et finale
    P(3,:) = 2 * P(2,:) - P(1,:) + [zdd1_0 / (n*(n-1)), zdd2_0 / (n*(n-1))];
    P(end-2,:) = 2 * P(end-1,:) - P(end,:) + [zdd1_f / (n*(n-1)), zdd2_f / (n*(n-1))];
    
    % Points de contrôle intermédiaires (linéaires pour simplifier)
    for i = 4:n-2
        P(i,:) = P(i-1,:) + (P(end,:) - P(1,:)) / (n+1);
    end
end
