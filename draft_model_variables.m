% ------------------------------------------------
%   draft arm model variables
% ------------------------------------------------

% Maxon motor: EC-i 30 30mm (shaft diameter = 4mm)

% variables from data sheet:
V_nom = 24;         % [V]
R = 0.713;           % [Ohm]
L = 0.475*1e-3;     % [H]
km = 22.8*1e-3;     % [Nm/A]
ka = km;
Jrot = 13.8*0.0001; % [gm^2]
B = 7e-4;

% calculated values:
Jload = 23.44*0.0001; % calculated from solidworks model
J = Jrot + Jload;   % [gm^2]
J = J/1000;         % [Kgm^2]

% gear ratio
gear_i = 1;

% Amplifier transfer function
AmpNum = [2.149e7];
AmpDen = [1 655.7 5.52e6];

% Joint 1 PID
Kp1 = 1;
Ki1 = 0;
Kd1 = 2;

% Joint 2 PID
Kp2 = 1;
Ki2 = 0;
Kd2 = 2;

% Path input

time = 0:0.001:8;
path = trajectories(100, 8, 0.001)
X = 549*cos(time/2);
Y = 549*sin(time/2);