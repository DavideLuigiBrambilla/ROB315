function J = CalculJacobienne(alpha, d, theta, r)
    rE = 0.1;    
    [g_06,g_elem] = CalculMGD(alpha, d, theta, r);
    g_6E = CalculTransformationElem(0,0,0,rE);
    g_0E = g_06 * g_6E;   

    N = length(theta);      % nombre d'axes du robot
    J = zeros(6,N);         % Initialisation de la matrice jacobienne 
    z_i = [0 0 1]';         %Vecteur unitaire portant l'articulation i

    for i=1:N
        p_iE = zeros(3,1);
        g_0i = eye(4);
        for j=1:i
            g_0i = g_0i*g_elem{j};
        end
 
        %Calcul de R_0i
        R_0i = g_0i(1:3,1:3);

        %calcul de p_iE (vecteur de translation du repere R_i � R_E)
        p_0E = g_0E(1:3,4);

        p_0i = g_0i(1:3,4);
        p_iE = p_0E - p_0i;

        %Calcul de Ji dans le repere R_0
        oJi = [cross(R_0i*z_i,p_iE) ; R_0i*z_i];

        %Concatenation de la matrice J
        J(1:6,i) = oJi;
    end
end