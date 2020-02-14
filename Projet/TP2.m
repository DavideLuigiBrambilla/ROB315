%% Parametres generales
close all
clear all
clc

% Initalisation Parametres
q = [-pi/2 0 -pi/2 -pi/2 -pi/2 -pi/2]';
dq = [0.5 1 -0.5 0.5 1 -0.5]';
% Convention DHM
d3 = 0.7;
r1 = 0.5;
r4 = 0.2;
alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
d = [0 0 d3 0 0 0];
theta = [q(1) q(2) pi/2+q(3) q(4) q(5) q(6)];
r = [r1 0 0 r4 0 0];
rE = 0.1;

% Initalisation Parametres 2
m = [15 10 1 7 1 0.5];
x_G1 = 0; y_G1 = 0; z_G1 = -0.25;
x_G2 = 0.35; y_G2 = 0; z_G2 = 0;
x_G3 = 0; y_G3 = -0.1; z_G3 = 0;
x_G4 = 0; y_G4 = 0; z_G4 = 0;
x_G5 = 0; y_G5 = 0; z_G5 = 0;
x_G6 = 0; y_G6 = 0; z_G6 = 0;
x_G = [x_G1 x_G2 x_G3 x_G4 x_G5 x_G6]';
y_G = [y_G1 y_G2 y_G3 y_G4 y_G5 y_G6]';
z_G = [z_G1 z_G2 z_G3 z_G4 z_G5 z_G6]';

%% Q12 - MatriceJacobienneGi
[OJv_Gi, OJ_wi] = CalculMatriceJacobienneGi(alpha, d, theta, r, x_G, y_G, z_G);
OVGi = [];
OwGi = [];

for i = 1:6
OVGi = [OVGi OJv_Gi(:,:,i) * dq];
OwGi = [OwGi OJ_wi(:,:,i) * dq];
end

% Les vitesses
OVGi
OwGi
%% Q13 - Matrice d'Inertie
A = CalculMatriceInertie(q)

%% Q14: Borne de la Matrice A

% but�es articulaires qmin et qmax d�finies � la question Q10
qmin = [-pi -pi/2 -pi -pi -pi/2 -pi]';
qmax = [0 pi/2 0 pi/2 pi/2 pi/2]';

% Calcul des bornes pour la matrice A
[mu1,mu2] = FindBorneA(A, qmin, qmax, 1000)

%% Q15: Calcule du couple de gravit�
G = CalculCoupleGravite(q)

%% Q16: Calcule d'un majorant pour G

% but�es articulaires qmin et qmax d�finies � la question Q10
qmin = [-pi -pi/2 -pi -pi -pi/2 -pi]';
qmax = [0 pi/2 0 pi/2 pi/2 pi/2]';

% Calcul du majorant pour de G
maxG = FindMajorantG (G, qmin,qmax, 1000)

%% Q17: Mod�le dynamique direct sur simulink
% Integr�e a la question 20

%% Q18: �valuation des param�tres de la g�n�ration de la trajectoire

% Configurations articulaires d�sir�es
qdi = [-1 0 -1 -1 -1 -1]';
qdf = [0 1 0 0 0 0]';

% Parametres
Te = 0.001;
Rred = [100 100 100 70 70 70];

% Couples moteurs maximale
tau = 5;

% Vecteur des accelerations articulaires maximales
ka = ((Rred * tau) / mu2)';
D = abs(qdf - qdi);

% Calcul des temps
tf = (sqrt((10/sqrt(3))*D./ka))
tf = max(tf)

%% Q19
SimOut=sim('ModeleSimulinkQ19_2016a.slx');

%% Q20
SimOut=sim('ModeleSimulinkQ20_2016a.slx');
