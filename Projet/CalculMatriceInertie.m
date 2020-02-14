function A = CalculMatriceInertie(q)
    %Calcul des parametres
    m=[15 10 1 7 1 0.5];
    
    x_G1 = 0; y_G1 = 0; z_G1 = -0.25;
    x_G2 = 0.35; y_G2 = 0; z_G2 = 0;
    x_G3 = 0; y_G3 = -0.1; z_G3 = 0;
    x_G4 = 0; y_G4 = 0; z_G4 = 0;
    x_G5 = 0; y_G5 = 0; z_G5 = 0;
    x_G6 = 0; y_G6 = 0; z_G6 = 0;
    x_G = [x_G1 x_G2 x_G3 x_G4 x_G5 x_G6]';
    y_G = [y_G1 y_G2 y_G3 y_G4 y_G5 y_G6]';
    z_G = [z_G1 z_G2 z_G3 z_G4 z_G5 z_G6]';

    iI(:,:,1)=[0.80 0 0.05 ; 0 0.80 0 ; 0.05  0  0.10];
    iI(:,:,2)=[0.10  0  0.10; 0  1.50 0 ; 0.10  0  1.50];
    iI(:,:,3)=[0.05 0 0 ; 0 0.01 0 ; 0 0 0.05];
    iI(:,:,4)=[0.50 0 0 ; 0 0.50 0 ; 0 0 0.05];
    iI(:,:,5)=[0.01 0 0 ; 0 0.01 0 ; 0 0 0.01];
    iI(:,:,6)=[0.01 0 0 ; 0 0.01 0 ; 0 0 0.01];
    
    d = [0 0 0.7 0 0 0];
    r = [0.5 0 0 0.2 0 0];
    alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
    theta = [q(1) q(2) q(3)+pi/2 q(4) q(5) q(6)]';
    
    Rred = [100 100 100 70 70 70];
    Jm = 1e-5*[1 1 1 1 1 1];
    
    %initialisation des tableaux
    g_0i = eye(4);
    Ig = zeros(3*size(q,1),3);
    A = zeros(size(q,1),size(q,1));
    
    %Calcul des matrices jacobiennes de chacun corps
    [OJv_Gi, OJ_wi] = CalculMatriceJacobienneGi(alpha, d, theta, r, x_G, y_G, z_G);
    
    for i = 1 : length(q)
        % Th�or�me  de Huygens generalis�
        Huygens = [((y_G(i))^2+(z_G(i))^2) -(x_G(i)*y_G(i)) -(x_G(i)*z_G(i));...
                -(x_G(i)*y_G(i)) ((x_G(i))^2+(z_G(i))^2) -(y_G(i)*z_G(i));...
                -(x_G(i)*z_G(i)) -(y_G(i)*z_G(i)) ((x_G(i))^2+(y_G(i))^2);];
    
        %Transport du tenseur d'inertie dans le rep�re R0
        g_elem = CalculTransformationElem(alpha(i),d(i),theta(i),r(i));
        g_0i = g_0i*g_elem;
        ROi = g_0i(1:3,1:3);
        
        % Application theoreme de Huygens generalis�
        iI_Gi = iI(:,:,i) - m(i) * Huygens;
        
        % Retour dans le repere qui nous interesse
        OI_Gi = ROi * iI_Gi * ROi';
        
        % Calcul matrice d'inertie du manipulateur
        A = A + ((m(i)*OJv_Gi(:,:,i)')*OJv_Gi(:,:,i) + (OJ_wi(:,:,i)')*OI_Gi*OJ_wi(:,:,i));
    end
    
    %Ajout des contributions des inerties des actionneurs
    A = A + diag(Rred.^2.*Jm);
    
end

