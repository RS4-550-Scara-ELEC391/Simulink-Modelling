% ------------------------------------------------
%   draft arm model variables
% ------------------------------------------------

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
length = 0.275;
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
K1 = 50;
Kp1 = 0.190;
Ki1 = 1;
Kd1 = 0.0090;

% Joint 2 PID
K2 = 100;
Kp2 = 0.1329;
Ki2 = 1;
Kd2 = 0.0044;

% sampling frequency
CF = 1163;  % [Hz]
N = 100;

% Path input
time = 0:0.001:8;
X = 200*cos(time)+320;
Y = 200*sin(time);