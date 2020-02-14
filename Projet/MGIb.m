function q_etoile = MGIb(Xd, q, kmax, epsx, aH, qmin, qmax)
    % Parametres
    rE = 0.1;
    r4 = 0.2;
    r1 = 0.5;
    r = [r1 0 0 r4 0 0];
    d3 = 0.7;
    d = [0 0 d3 0 0 0];
    alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
    theta = [q(1) q(2) pi/2+q(3) q(4) q(5) q(6)];
    k = 0;

    while true
        k = k+1;

        J = CalculJacobienne(alpha, d, theta, r);
        Ji = J(1:3, :);
        [g_06, ~] = CalculMGD(alpha, d, theta, r);
        g_0E = g_06*CalculTransformationElem(0,0,0,rE);
        X = g_0E(1:3, 4);

        dX = Xd - X;

        % Calcul de la fonction objectif
        qbar = (qmax-qmin)/2;

        H = [];
        for i = 1:size(q,1)
            H = [H; 2*(q(i)-qbar(i))/(qmax(i)-qmin(i))^2];
        end

        % Solution de l'optimisation quadratique
        q = q + (pinv(Ji) * dX) + (eye(size(Ji,2))-(pinv(Ji)*Ji))*(-aH * H')';
        error = norm(dX, 2);

        % Condition de fin de la boucle
        if (error < epsx) || (k > kmax)
            break
        end
    end
    q_etoile = q;
end

