%------------------------------------------------
%   draft arm model variables
%------------------------------------------------

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

% P-Controller:
Kp = 1;
Ki = 0;
Kd = 2;