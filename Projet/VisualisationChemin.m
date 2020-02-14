function VisualisationChemin(q, Xdk, samples)
    % Définition des parametres
    N = size(q,2);
    plotStep = round(N/samples);
    
    % Affiches une <samples> quantité de configurations du robot
    for i = 1:N
        if (mod(i,plotStep)) == 1
            if i == 1
                % Affiche la premiere configurations avec la couleur verte
                VisualisationRepere(q(:,i)', 0, 'green');
                else
                VisualisationRepere(q(:,i)', 0, 'black');
            end
        end
    end
    % Plots la droite suivi
    plot3(Xdk(1,:),Xdk(2,:),Xdk(3,:),'m--', 'color', 'magenta');
end