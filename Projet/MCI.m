function [XdTot, qTot] = MCI(Xdi, Xdf, V, Te, qi)
% Parametres
kmax = 100;
epsx = 0.005;
aStep = 0.001;

% Transformations pour l'espace discretisé
distance = norm(Xdf - Xdi,2);
step = (V*Te)*(Xdf - Xdi)/distance; %Normalisé (vecteur unitaire)
temps = distance/V;

% Initialisations des variables
Xd = Xdi;
XdTot = Xdi;
q = qi;
qTot = qi;
t = 0;
    % Vérification de q a chaque 1ms
    while (t < temps)
        % Mis a jour de Xd e q
        Xd = Xd + step;
        q = MGI(Xd, q, kmax, epsx, aStep);

        % Sauvegarde des positions et des configurations
        XdTot = [XdTot Xd];
        qTot = [qTot q];
        
        % Incrementation du temps
        t = t+1e-3;
    end
end