% ------------------------------------------------
%   draft arm model variables
% ------------------------------------------------

addpath("Path_planning\");

% Maxon motor: EC-i 30 30mm (shaft diameter = 4mm)

% Maxon motor variables
V_nom = 12;         % [V]
B = 7e-4;

% Motor 1 (148866):
R1 = 0.115;        % [Ohm]
L1 = 0.245e-3;     % [H]
km1 = 16.4e-3;     % [Nm/A]
ka1 = km1;

% Motor 2 (268193):
R2 = 0.2;           % [Ohm]
L2 = 0.0345e-3;     % [H]
km2 = 13.9e-3;      % [Nm/A]
ka2 = km2;

% robot
armlength = 0.275;
gear1 = 120;
gear2 = 20;    % --> double check

% Calculated values:
J1 = 0.5468;            % [Kgm^2]
J2 = 0.0492;

% Amplifier transfer function
AmpNum = [1.393e9];
AmpDen = [1 1.434e4 5.524e8];

s = tf('s');

% Joint 1 PID
K1 = 60;
Kp1 = 0.3;      %0.18;
Ki1 = 0.7;
Kd1 = 0.03;

% Joint 2 PID
K2 = 40;
Kp2 = 0.22;
Ki2 = 1.1;
Kd2 = 0.04;
% Kd2 = 0.04;

% sampling frequency
CF = 1163;  % [Hz]
N = 100;

% Path input
time = 0:0.001:8;
% X = 200*cos(time)+320;
% Y = 200*sin(time);

        % Pentagonal
% X = [trajectories(550, 400, 1, 0.001) 400*ones(1,50) trajectories(400, 175, 0.5, 0.001) 175*ones(1,50) trajectories(175, 175, 1.5, 0.001) 175*ones(1,100) trajectories(175, 400, 0.5, 0.001) 400*ones(1,100) trajectories(400, 550, 1, 0.001) 550*ones(1, 8001-4800)]; 
% Y = [trajectories(0, 250, 1, 0.001) 250*ones(1,50) trajectories(250, 200, 0.5, 0.001) 200*ones(1,50) trajectories(200, -200, 1.5, 0.001) -200*ones(1,100) trajectories(-200, -250, 0.5, 0.001) -250*ones(1,100) trajectories(-250, 0, 1, 0.001) zeros(1, 8001-4800)];

        % Diamond
X = [trajectories(550, 400, 1, 0.001) 400*ones(1,100) trajectories(400, 250, 1, 0.001) 250*ones(1,100) trajectories(250, 400, 1, 0.001) 400*ones(1,100) trajectories(400, 550, 1, 0.001) 550*ones(1, 8001-4300)]; 
Y = [trajectories(0, 250, 1, 0.001) 250*ones(1,100) trajectories(250, 0, 1, 0.001) 0*ones(1,100) trajectories(0, -250, 1, 0.001) -250*ones(1,100) trajectories(-250, 0, 1, 0.001) zeros(1, 8001-4300)];

        % Diamond (slow)
% X = [trajectories(550, 400, 1.5, 0.001) 400*ones(1,50) trajectories(400, 250, 1.5, 0.001) 250*ones(1,50) trajectories(250, 400, 1.5, 0.001) 400*ones(1,50) trajectories(400, 550, 1.5, 0.001) 550*ones(1, 8001-6150)]; 
% Y = [trajectories(0, 250, 1.5, 0.001) 250*ones(1,50) trajectories(250, 0, 1.5, 0.001) 0*ones(1,50) trajectories(0, -250, 1.5, 0.001) -250*ones(1,50) trajectories(-250, 0, 1.5, 0.001) zeros(1, 8001-6150)];

        % Homing (angles)
% q1 = [trajectories(0, 0.436, 1, 0.001) 0.436*ones(1,100) trajectories(0.436, 0, 2.2, 0.001) zeros(1, 2400) zeros(1, 8001-5700)];
% q2 = [trajectories(0, 1.66, 1, 0.001)  1.66*ones(1,100+2200) trajectories(1.66, 0, 2.4, 0.001) zeros(1, 8001-5700)];

        % rotate 90deg
% X = [trajectories(550, 0, 0.5, 0.001) zeros(1,8001-500)];
% Y = [trajectories(0, 550, 0.5, 0.001) 550*ones(1,8001-500)];

size(X)
close

