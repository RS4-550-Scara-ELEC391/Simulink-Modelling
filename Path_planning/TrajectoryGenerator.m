% ---------------------------------------------------
%  Generates Trajectory for Scara Joint 1 and 2 
%  Note: incomplete (Mar. 3)
% ---------------------------------------------------

function TrajectoryGenerator(Od)

close all

% Od = desired end-effector location as a column vector
% Cd = desired frame


% known initial pose:
Oi = [471.5; 149.1; 433.1];

% basis vectors:
i = [1;0;0];
j = [0;1;0];
k = [0;0;1];

% time values:
% total time = 1000ms
  dt = 5; 
% number of data points = 200

% ------------------------------------------------------
%           Finding Gripper Pose Trajectories
% ------------------------------------------------------

% to compute linear and angular pose trajectories, we need total linear 
% and angular distances:

% preallocate linear pose trajectories
lindisp = zeros(3,200);
linvel = zeros(3,200);
linaccel = zeros(3,200);

angvel = zeros(3,200);
angaccel = zeros(3,200);


% find x distance
    dx = Od(1) - Oi(1)
    [lindisp(1,:), linvel(1,:), linaccel(1,:),t] = trajectories(dx);
% find y distance
    dy = Od(2) - Oi(2)
    [lindisp(2,:), linvel(2,:), linaccel(2,:)] = trajectories(dy);
% find z distance
    dz = Od(3) - Oi(3)
    [lindisp(3,:), linvel(3,:), linaccel(3,:)] = trajectories(dz);




% --- Plot Gripper Pose Trajectories ---
figure
poseplot = tiledlayout(3,1,'padding', 'compact');
title(poseplot, 'Pose Trajectories')
nexttile;
plot(t,lindisp(1,:),t,lindisp(2,:),t,lindisp(3,:));
title('linear displacement')
legend('x','y','z')

nexttile;
plot(t,linvel(1,:),t,linvel(2,:),t,linvel(3,:));
title('linear velocity')
legend('x','y','z')

nexttile;
plot(t,linaccel(1,:),t,linaccel(2,:),t,linaccel(3,:));
title('linear acceleration')
legend('x','y','z')


% ------------------------------------------------------
%           Finding Joint Trajectories
% ------------------------------------------------------
    
 % initial joint vector
 qi = [0; 0];
 
 % pre-allocate vectors and assign q(0)
 q = zeros(2,200);
 q(:,1) = qi;
 w = zeros(2,200);
 wdot = zeros(2,200);
 
%  % incrementally calculate q, omega and omega-dot
%  for n = 2:200
%      
%      % Joint velocity - equation (25)
%  w(:,n) = (pinv(Scara_J(q(:,n-1))))*[linvel(:,n-1); angvel(:,n-1)];
% 
%     % Joint displacement - Newton-Raphson method
%  q(:,n) = q(:,n-1) + (5)*w(:,n-1);
%  
%     % Joint acceleration - equation (26)
%  wdot(:,n) = (pinv(Scara_J(q(:,n-1)))*([linaccel(:,n-1); angaccel(:,n-1)] - ((Scara_J(q(:,n))-Scara_J(q(:,n-1)))/5)*w(:,n-1)));
%  
%  end

% -> Use inverse kin to find joint displacements q

    for n = 2:200

        [temp1_1,temp2_1, ~,~] = xy2theta(275,275,lindisp(1,n), lindisp(2,n));
        q(:,n) = [temp1_1; temp2_1];


    end

    dq = gradient(q);
    w = dq/dt;


 % --- Plot Joint Trajectories --- 
 
 figure
 jointplot = tiledlayout(1,3,'Padding', 'compact');
 title(jointplot, 'Joint Trajectories')
 nexttile;
 hold on
 plot(t, q(1,:))
 plot(t, q(2,:))
 legend('q1','q2','Location','northwest')
 legend('boxoff')
 title('Joint Displacement, q (deg)')
 hold off
 
 nexttile;
 
 hold on
 plot(t, w(1,:))
 plot(t, w(2,:))
 legend('\omega1','\omega2','Location','northwest')
 legend('boxoff')
 title('Joint Velocity, \omega (deg/ms)')
 hold off
 
 nexttile;
 
 hold on
 plot(t, wdot(1,:))
 plot(t, wdot(2,:))
 legend('d\omega1/dt','d\omega2/dt','Location','northwest')
 legend('boxoff')
 title('Joint Acceleration, d\omega/dt (deg/ms^{2})')
 hold off
 

    % check final gripper pose

    T01 = DH_homog(q(1,end), 200, 0, 0);
    T12 = DH_homog(q(2,end), 0, 275, 0);
    T23 = DH_homog(0, 50, 275, 0);
    
    % Homogenous transformation matrix
    T_final = T01*T12*T23
    
    
end %function




