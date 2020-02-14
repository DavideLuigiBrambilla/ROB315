function VisualisationEllipsoides(J, q, g_0E)
    % Décomposition en valeurs singuliers
    [U,S,V] = svd(J*J.');
    
    % Ellipsoide des vitesses
    AxisValues = sqrt(diag(S));

    % Visualisation du Robot et les rèperes
    VisualisationRepere(q, 0, 'black');
    
    %Rotation (De la Q4) Axe et angle de rotation équivalent à une rotation donnée
    pf = g_0E(1:3,4);
    [q_r,n] = AngleAxe_from_Rot (V);
    
    % Ellipsoïdes de vitesse
    % [x,y,z] = 	(xc,yc,zc,xr,yr,zr,n)
    [x,y,z] = ellipsoid(pf(1),pf(2),pf(3),AxisValues(1),AxisValues(2),AxisValues(3));
    Ellipsoide = surf(x,y,z);
    alpha 0.7;
    Ellipsoide.EdgeColor = 'none';
    rotate(Ellipsoide,n,radtodeg(q_r),pf);
end
