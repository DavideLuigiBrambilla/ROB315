%% Parametres generales
close all
clear all
clc
% Configurations articulaires possibles
qi = [-pi/2 0 -pi/2 -pi/2 -pi/2 -pi/2]';
qf = [0 pi/4 0 pi/2 pi/2 0]';

% Configuration articulaire choisi
q = qi;

% Convention DHM
d3 = 0.7;
r1 = 0.5;
r4 = 0.2;
alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
d = [0 0 d3 0 0 0];
theta = [q(1) q(2) pi/2+q(3) q(4) q(5) q(6)];
r = [r1 0 0 r4 0 0];
rE = 0.1;

%% Q1 et Q2
% Sur le rapport

%% Q3: MGD du robot
g_6E = CalculTransformationElem(0,0,0,rE);
[g_06, g_element] = CalculMGD(alpha, d, theta, r);
g_0E = g_06 * g_6E

%% Q4: Axe et angle de rotation �quivalent � une rotation donn�e

% Vecteur de position
P_0E = g_0E(1:3, 4)
% Matrice de rotation
R_0ER = g_0E(1:3, 1:3);
% Trouve l'angle q et l'axe n a partir d'une rotation donn�e
[ang_q,n] = AngleAxe_from_Rot (R_0ER)


%% Q5: Visualisation des repere du robot
figure('Name','Question 05: Visualisation des repere du robot','NumberTitle','off')
VisualisationRepere(q, 1, 'black');
title('Visualisation des rep�res du robot');

%% Q6: Calcul de la Jacobienne et torseurs cin�matiques

% Vitesse articulaire
q_point = [0.5 1 -0.5 0.5 1 -0.5]';

% Calcula de la Jacobienne
J = CalculJacobienne(alpha, d, theta, r)
V0EOE = J * q_point

    
%% Q7: Transmission des vitesses

    %  Limitation de J � les vitesses de translation
    J7 = J(1:3,:)
    
    % D�composition en valeurs singuliers
    [U,S,V] = svd(J7*J7.')
    
    %Direction privil�gi�
    [m,idx] = max(diag(S));
    Direction_privilegie = V(:,idx);
    
    % Calcul de la manipulabilit�
    eigenvalues = sqrt(diag(S));
    manipulabilite = prod(eigenvalues);
    
    %Qualit� de la transmission de la vitesse
    VisualisationEllipsoides(J(1:3,:), q, g_0E);
    title('Ellipso�de de vitesse');


%% Q8.1 : Calculer modele geometrique inverse

% Parametres du model
X_di = [-0.1 -0.7 0.3]';
q0 = [-1.57 0 -1.47 -1.47 -1.47 -1.47]';
kmax = 100;
aStep = 0.005;
epsx = 0.001;

% Calcul de q*
q_etoile1 = MGI(X_di, q0, kmax, epsx, aStep)

% Verification du resultat 8.1
theta_etoile1 = [q_etoile1(1) q_etoile1(2) pi/2+q_etoile1(3) q_etoile1(4) q_etoile1(5) q_etoile1(6)];
[g_06_etoile1, g_elem_etoile1] = CalculMGD(alpha, d, theta_etoile1, r);
g_0E_etoile1 = g_06_etoile1*CalculTransformationElem(0,0,0,rE);
diff1 = abs(X_di - g_0E_etoile1(1:3,4))

%% Q8.2 : Calculer modele geometrique inverse

% Parametres du model
X_df = [0.64 -0.1 1.14]';
q0 = [0 0.8 0 1 2 0]';
kmax = 100;
aStep = 0.005;
epsx = 0.001;
q_etoile2 = MGI(X_df, q0, kmax, epsx, aStep)

% verification du resultat 8.2
theta_etoile2 = [q_etoile2(1) q_etoile2(2) pi/2+q_etoile2(3) q_etoile2(4) q_etoile2(5) q_etoile2(6)];
[g_06_etoile2, g_elem_etoile2] = CalculMGD(alpha, d, theta_etoile2, r);
g_0E_etoile2 = g_06_etoile2*CalculTransformationElem(0,0,0,rE);
diff2 = abs(X_df - g_0E_etoile2(1:3,4))


%% Q9: Trajectoire � suivre 
%Parametres donn�es dans l'�nonc� de la question
X_di = [-0.1 -0.7 0.3]';
X_df = [0.64 -0.1 1.14]';
V = 1;
Te = 1e-3;

% Execute la fonction de suivi de trajectoire
[Xtot, qTot] = MCI(X_di,X_df,V,Te,qi);

% Plot les r�sultats obtenus
figure('Name','Question 09: Trajectoire � suivre','NumberTitle','off')
VisualisationChemin(qTot, Xtot, 6);
title('Chemin suivi (mouvement rectiligne)');
%% Q10
% afficher les differents courbes
qmin = [-pi -pi/2 -pi -pi -pi/2 -pi]';
qmax = [0 pi/2 0 pi/2 pi/2 pi/2]';

% Plot graphique avec l'�volution de chaque variable articulaire
figure('Name','Question 10: �volution des variables articulaires','NumberTitle','off')
plotEvolution(qTot, qmin, qmax)
%% Q11: �loignement des but�es articulaires

% Parametres
X_di = [-0.1 -0.7 0.3]';
X_df = [0.64 -0.1 1.14]';
V = 1;
Te = 0.001;
qmin = [-pi -pi/2 -pi -pi -pi/2 -pi]';
qmax = [0 pi/2 0 pi/2 pi/2 pi/2]';

% Chemin a suivre avec constraint
[Xtot, qTot] = MCIbutees(X_di, X_df, V, Te, qi, qmin, qmax);

% Plot graphique avec l'�volution de chaque variable articulaire
figure('Name','Question 11: �volution des variables articulaires','NumberTitle','off')
plotEvolution(qTot, qmin, qmax)

figure('Name','Question 11: Visualisation du Ch�min','NumberTitle','off')
VisualisationChemin(qTot, Xtot, 6);
title('Chemin suivi (mouvement rectiligne avec constraint)');
