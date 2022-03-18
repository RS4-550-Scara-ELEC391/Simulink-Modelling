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

% Calculated values:
% Jload = 23.44*0.0001; % calculated from solidworks model
% J = Jrot + Jload;   % [gm^2]
% J = J/1000;         % [Kgm^2]
%J1 = (Jrot + mass*length^2)/1000 + 1.29;
J1 = 2.60;            % from solidworks with 1kg load
J2 = Jrot/1000 + 0.09;

% Amplifier transfer function
AmpNum = [2.149e7];
AmpDen = [1 655.7 5.52e6];

s = tf('s');

% --- Joint 1 PID ---
Kp1 = 2;
Ki1 = 0;
Kd1 = 3;

% --- Joint 2 PID ---
Kp2 = 1;
Ki2 = 0;
Kd2 = 2;

% sampling frequency
CF = 1000;              % to be found from arduino code
N = 2*CF;

% Path input
time = 0:0.001:8;
X = 200*cos(time)+340;
Y = 200*sin(time);




%% Transfer function

PID1 = tf(Kp1 + Ki1/s + Kd1*N*s/(s+N));
Amplifier = tf(AmpNum, AmpDen);

% motor and mech gain
Gm1 = km*gear_i* tf(1,[L R])*tf(1,[J1 B]);
Hm1 = gear_i*ka;
Motormech1 = Gm1 / (1+Gm1*Hm1);

% forward path gain G1
G1 = Amplifier*Motormech1*tf(1/s);

% feedback path gain H1
H1 = tf(N/(s+N));

% closed loop transfer function, joint 1
cltf1 = G1/(1+G1*H1)









% % Transfer function
% syms s
% Hin = gear_i*ka;
% Gin = km*gear_i/((L*s+R)*(J1*s+B));
% Tin = Gin / (1+Gin*Hin);
% 
% Hout = 1;
% PID = K1*(Kp1 + Ki1/s + Kd1*s);
% Gout = (5000/(s+5000))*Tin*(1/s);
% Tout = Gout / (1+Gout*Hout)
% 
% simplify(Tout)
