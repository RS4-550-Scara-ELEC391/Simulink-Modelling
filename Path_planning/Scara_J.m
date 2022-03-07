function J = Scara_J(q)

    % -- Part B --

    % -- Transformation matrices between links --
    % input the given joint angles and DH parameters
        % DH_homog(theta, di, ai, alpha)
    T01 = DH_homog(q(1), 200, 0, 0);
    T12 = DH_homog(q(2), 0, 275, 0);
    T23 = DH_homog(0, 50, 275, 0);
    
    % Homogenous transformation matrix
    % T = T01*T12*T23;

    
    % -- Jacobian --
    
    % calculate origins O0, O1, O2, O3:
    
    % set O0 to be [0;0;0]
    O0 = [0;0;0];
    
    % find the following coord origins with respect to the base origin
    % ex) O1 = O0 + d (from T01 matrix)
    %     O2 = O1 + 0C1 * d (from T12 matrix)
    
    O1 = O0 + T01(1:3, end);
    O2 = O1 + T01(1:3, 1:3)*T12(1:3, end);
    O3 = O2 + T01(1:3, 1:3)*T12(1:3, 1:3)*T23(1:3, end);
   
    % matrix of origin coordinates wrt base
    % origins = ([O0 O1 O2 O3]);
    
    % Calculate k1 to k3 wrt the base system:
    k0 = [0;0;1];
    
    %k1 = C1*k = C0*0C1*k (since C0 is I, k1 = 0C1*k)
    k1 = T01(1:3, 1:3)*[0;0;1];
    k2 = T01(1:3, 1:3)*T12(1:3, 1:3)*[0;0;1];
    k3 = T01(1:3, 1:3)*T12(1:3, 1:3)*T23(1:3, 1:3)*[0;0;1];
  
    
    % Calculate Jacobian
    J = [cross(k0,(O3-O0)) cross(k1,(O3-O1)) cross(k2,(O3-O2)) cross(k3,(O3-O3));
        k0 k1 k2 k3];

end