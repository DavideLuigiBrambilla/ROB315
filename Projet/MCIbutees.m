function [XdTotbutee, qTotbutee] = MCIbutees(Xdi, Xdf, V, Te, qi, qmin, qmax)

    % Parametres
    kmax = 100;
    epsx = 0.005;
    aStep = 0.001;
    aH = 0.001;

    % Transformations pour l'espace discretis�
    distance = norm(Xdf - Xdi,2);
    step = (V*Te)*(Xdf - Xdi)/distance; %Normalis� (vecteur unitaire)
    temps = distance/V;

    % Initialisations des variables
    Xd = Xdi;
    XdTotbutee = Xdi;
    q = qi;
    qTotbutee = qi;
    t = 0;
        % V�rification de q a chaque 1ms
        while (t < temps)
            % mise a jour de la position
            Xd = Xd + step;

            %calcul de la configuration li�e a la position courante
            q = MGIb(Xd, q, kmax, epsx, aH, qmin, qmax);

            % sauvegarde des positions et des configurations
            XdTotbutee = [XdTotbutee Xd];
            qTotbutee = [qTotbutee q];

            % Incrementation du temps
            t = t+1e-3;
        end
end