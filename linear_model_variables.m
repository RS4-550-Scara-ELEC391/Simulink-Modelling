%------------------------------------------------
%   Linear model variables
%------------------------------------------------

% Maxon motor: EC

% variables from data sheet:
V_nom = 48;         % [V]
R = 0.345;           % [Ohm]
L = 0.273*1e-3;     % [H]
km = 84.9*1e-3;     % [Nm/A]
ka = km;
Jrot = 831*0.0001; % [gm^2]
B = 7e-4;
mass = 2450*0.001;        % [kg]

% robot
length = 0.275;
gear_i = 1;

% % calculated values:
% Jload = 23.44*0.0001; % calculated from solidworks model
% J = Jrot + Jload;   % [gm^2]
% J = J/1000;         % [Kgm^2]

J1 = (Jrot + mass*length^2)/1000 + 1.29;

J2 = Jrot/1000 + 0.09;

% --- Joint 1 ---
% P-Controller:
z1 = 5000;
p1 = 5000;
K1 = 1.5;
Kp1 = 1;
Ki1 = 0;
Kd1 = 1/z1 - 1/p1;


% --- Joint 2 ---
% P-Controller:
z2 = 5000;
p2 = 5000;
K2 = 1.5;
Kp2 = 1;
Ki2 = 0;
Kd2 = 1/z2 - 1/p2;


% Transfer function
syms s
Hin = gear_i*ka;
Gin = km*gear_i/((L*s+R)*(J1*s+B));
Tin = Gin / (1+Gin*Hin);

Hout = 1;
PID = K1*(Kp1 + Ki1/s + Kd1*s);
Gout = (5000/(s+5000))*Tin*(1/s);
Tout = Gout / (1+Gout*Hout)

simplify(Tout)
