function Gamma = CalculCoupleFrottement(dq_dt)
    
    %parametres
    Fv = 10*ones(6,1);
    
    %Fonction donnée à l'interieure du TP
    Gamma = diag(dq_dt)*Fv;
end