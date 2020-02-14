function VisualisationRepere(q, labelOn, rep_color)
    % Parametres
    r4 = 0.2;
    r1 = 0.5;
    r = [r1 0 0 r4 0 0];
    d3 = 0.7;
    d = [0 0 d3 0 0 0];
    alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
    rE = 0.1;
    theta = [q(1) q(2) pi/2+q(3) q(4) q(5) q(6)];
    
    % MGD du robot
    g_6E = CalculTransformationElem(0,0,0,rE);
    [g_06,g_element] = CalculMGD(alpha, d, theta, r);

    %Point de départ: (0,0,0)
    origin = [0;0;0] ; 
    % q = 0 pour les matrices de Rotation Rxq, Ryq et Rzq initialement
    RotN = eye(3) ;
    
    % Plot R0
    plot_repere(origin, RotN, 'blue','o', labelOn, rep_color) ;
    
    % Plot Articulations et RE
    plot_articulation(origin, RotN, g_element, g_6E, labelOn, rep_color);
    
    
    xlabel('x');
    ylabel('y');
    zlabel('z');
    grid on;
    axis equal;
    view(-135,45)  
end

function plot_articulation(origin, RotN, g_element, g_6E, labelOn, rep_color)
    position = origin;
    for i = 1:length (g_element)
        position = origin +  RotN*g_element{i}(1:3,4);
        RotN = RotN*g_element{i}(1:3,1:3);
        plot3([origin(1) position(1)],[origin(2) position(2)],[origin(3) position(3)],rep_color,'LineWidth',2);
        hold on
        scatter3(origin(1),origin(2),origin(3),rep_color,'filled');
        hold on
        scatter3(position(1),position(2),position(3),rep_color,'filled');
        hold on
        origin = position;
    end

    % Position et orientation du repère terminal
    position = origin +  RotN*g_6E(1:3,4); 
    RotN = RotN*g_6E(1:3,1:3);

    %Plot the terminal
    plot3([origin(1) position(1)],[origin(2) position(2)],[origin(3) position(3)],rep_color,'LineWidth',2);
    hold on
    scatter3(position(1),position(2),position(3),'red','filled');
    hold on

    %Plot terminal
    plot_repere(position, RotN, 'red', 'e', labelOn, rep_color)
end

function plot_repere(origin, RotN, c, id, labelOn, rep_color)
    point = origin;
    rotation = RotN;
    x = [1;0;0];
    y = [0;1;0];
    z = [0;0;1];
    O1 = 0.15 * rotation * x;
    O2 = 0.15 * rotation * y;
    O3 = 0.15 * rotation * z;
    plot3([point(1) point(1)+O1(1)],[point(2) point(2)+O1(2)],[point(3) point(3)+O1(3)],c,'LineWidth',1);
    hold on
    if labelOn
        text(0.05 +point(1)+O1(1),0.05 +point(2)+O1(2),0.05 +point(3)+O1(3), strcat('x',id), 'Color', c);
    end
    
    
    plot3([point(1) point(1)+O2(1)],[point(2) point(2)+O2(2)],[point(3) point(3)+O2(3)],c,'LineWidth',1);
    hold on
    if labelOn
        text(0.05 +point(1)+O2(1),0.05 +point(2)+O2(2),0.05 +point(3)+O2(3), strcat('y',id), 'Color', c);
    end
    
    plot3([point(1) point(1)+O3(1)],[point(2) point(2)+O3(2)],[point(3) point(3)+O3(3)],c,'LineWidth',1);
    hold on
    if labelOn
        text(0.05 +point(1)+O3(1),0.05 +point(2)+O3(2),0.05 +point(3)+O3(3), strcat('z',id), 'Color', c);
    end 
end