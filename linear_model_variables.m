%------------------------------------------------
%   Linear model variables
%------------------------------------------------

% Maxon motor: 148866

% variables from data sheet:
V_nom = 12;         % [V]
R = 0.115;           % [Ohm]
L = 0.245e-3;     % [H]
km = 16.4e-3;     % [Nm/A]
ka = km;
B = 7e-4;
mass = 2450*0.001;        % [kg]

% robot
length = 0.275;
gear_i = 100;

% Calculated values:
J1 = 2.60;            % from solidworks with 1kg load
J2 = 0.50;

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
N = 100;

% Path input
time = 0:0.001:8;
X = 200*cos(time)+340;
Y = 200*sin(time);




%% Transfer function Joint 1

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
cltf1 = G1/(1+G1*H1);

% bodeplot(cltf1), grid on

[Gm,Pm,Wcg,Wcp] = margin(G1*H1); % gain margin, Phase margin, phase crossing freq, gain crossing freq

% returns the gain margin Gm in absolute units, the phase margin Pm, and 
% the corresponding frequencies Wcg and Wcp, of sys. Wcg is the frequency 
% where the gain margin is measured, which is a –180° phase crossing 
% frequency. Wcp is the frequency where the phase margin is measured, 
% which is a 0-dB gain crossing frequency

z = Wcg / 10;   % Initial zero 1 decade before PXO
p = 2*CF;
Kp1 = 2/z - 1/p;
Ki1 = 1;
Kd1 = 1/z^2 - Kp1/p;

% %New transfer function with PID (Iterate)
% for n=1:10
% PID1 = tf(Kp1 + Ki1/s + Kd1*N*s/(s+N));
% [Gm,Pm,Wcg,Wcp] = margin(PID1*G1*H1);
% z = Wcg / 10   % Initial zero 1 decade before PXO
% p = 2*CF;
% Kp1 = 2/z - 1/p;
% Ki1 = 1;
% Kd1 = 1/z^2 - Kp1/p;
% end

    % -> lowest solution z = 7.8349
z = 7.8349;
Kp1 = 2/z - 1/p;
Ki1 = 1;
Kd1 = 1/z^2 - Kp1/p;

PID1 = tf(Kp1 + Ki1/s + Kd1*N*s/(s+N));
%K1 = 1/abs(freqresp(PID1*G1*H1,z));

figure(1)
clf
rlocus(PID1*G1*H1)
K1 = 39.8;

[Gm,Pm,Wcg,Wcp] = margin(K1*PID1*G1*H1)
% bodeplot(K1*PID1*G1/(1+K1*PID1*G1*H1))

% step response of new closed loop system
%step(K1*PID1*G1/(1+K1*PID1*G1*H1)), grid on



%% Transfer function (Ten step proccess) Joint 2

% motor and mech gain
Gm2 = km*gear_i* tf(1,[L R])*tf(1,[J2 B]);
Hm2 = gear_i*ka;
Motormech2 = Gm2 / (1+Gm2*Hm2);

% forward path gain G1
G2 = Amplifier*Motormech2*tf(1/s);

% feedback path gain H1
H2 = tf(N/(s+N));

% closed loop transfer function, joint 1

[Gm,Pm,Wcg,Wcp] = margin(G2*H2); % gain margin, Phase margin, phase crossing freq, gain crossing freq


z = Wcg / 10;   % Initial zero 1 decade before PXO
p = 2*CF;
Kp2 = 2/z - 1/p;
Ki2 = 1;
Kd2 = 1/z^2 - Kp1/p;

% % New transfer function with PID (Iterate)
% for n=1:10
% PID2 = tf(Kp2 + Ki2/s + Kd2*N*s/(s+N));
% [Gm,Pm,Wcg,Wcp] = margin(PID2*G2*H2);
% z = Wcg / 10   % Initial zero 1 decade before PXO
% p = 2*CF;
% Kp2 = 2/z - 1/p;
% Ki2 = 1;
% Kd2 = 1/z^2 - Kp1/p;
% end

    % -> lowest solution z = 10.3828
z = 10.3828;
Kp2 = 2/z - 1/p;
Ki2 = 1;
Kd2 = 1/z^2 - Kp1/p;

K2 = 1/abs(freqresp(PID2*G2*H2,z));
PID2 = tf(Kp2 + Ki2/s + Kd2*N*s/(s+N));

figure(2)
clf
rlocus(PID2*G2*H2)
K2 = 1.25;

[Gm,Pm,Wcg,Wcp] = margin(K2*PID2*G2*H2);
% bodeplot(K2*PID2*G2/(1+K2*PID2*G2*H2))

% step response of new closed loop system
% step(K2*PID2*G2/(1+K2*PID2*G2*H2), 4), grid on


