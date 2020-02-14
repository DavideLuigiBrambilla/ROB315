function [mu1, mu2] = FindBorneA(A, qmin, qmax, NumTry)
    % Initialisations des bornes de la matrice A
    mu1 = inf;
    mu2 = 0;
    
    % Nous allons faire varier les q entre qmin et qmax
    dq = (qmax-qmin)/NumTry;
    qCur = qmin;

    for i = 1 : NumTry
        qCur = qCur + dq;
        A = CalculMatriceInertie(qCur);
        maxCur = max(eig(A));
        minCur = min(eig(A));
        if maxCur > mu2
            mu2 = maxCur;
        end
        if minCur < mu1
            mu1 = minCur;        
        end
    end
end