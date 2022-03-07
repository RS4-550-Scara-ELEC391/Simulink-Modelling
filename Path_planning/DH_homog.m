
function T = DH_homog(theta, di, ai, alpha)

    % Calculate rotation matrixes Q1 (about k) and Q2 (about i)
    Q1 = [cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];
    Q2 = [1 0 0; 0 cosd(alpha) -sind(alpha); 0 sind(alpha) cosd(alpha)];
    
    % Rotation matrix Q
    Q = Q1*Q2;
    
    % vector d
    d = (ai)*Q1*[1;0;0] + [0;0;di];
    
    % Homogenous Transformation Matrix
    T = [Q d; 0 0 0 1];

end

