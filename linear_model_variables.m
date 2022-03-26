%------------------------------------------------
%   Linear model variables
%------------------------------------------------

% Maxon motor joint 1: 148866

% variables from data sheet 
% Motor 1 (148866):
V_nom = 12;         % [V]
R1 = 0.115;           % [Ohm]
L1 = 0.245e-3;     % [H]
km1 = 16.4e-3;     % [Nm/A]
ka1 = km1;
B = 7e-4;

% Motor 2 (268193):
R2 = 0.2;           % [Ohm]
L2 = 0.0345e-3;     % [H]
km2 = 13.9e-3;     % [Nm/A]
ka2 = km2;

% robot
length = 0.275;
gear1 = 120;
gear2 = 200;    % --> double check

% Calculated values:
J1 = 2.60;            % from solidworks with 1kg load
J2 = 0.50;

% Amplifier transfer function
AmpNum = [2.149e7];
AmpDen = [1 655.7 5.52e6];

s = tf('s');

% Joint 1 PID
K1 = 0.589;
Kp1 = 20.9419;
Ki1 = 1;
Kd1 = 109.6356;

% Joint 2 PID
K2 = 0.0753;
Kp2 = 19.9995;
Ki2 = 1;
Kd2 = 99.9905;

% sampling frequency
CF = 1163;  % [Hz]
N = 100;

% Path input
time = 0:0.001:8;
X = 200*cos(time)+340;
Y = 200*sin(time);




%% Transfer function Joint 1

Amplifier = tf(AmpNum, AmpDen);

% motor and mech gain
Gm1 = km1*gear1* tf(1,[L1 R1])*tf(1,[J1 B]);
Hm1 = gear1*ka1;
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
Gm2 = km2*gear2* tf(1,[L2 R2])*tf(1,[J2 B]);
Hm2 = gear2*ka2;
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

PID2 = tf(Kp2 + Ki2/s + Kd2*N*s/(s+N));
% K2 = 1/abs(freqresp(PID2*G2*H2,z));

figure(2)
clf
rlocus(PID2*G2*H2)
K2 = 0.151;

[Gm,Pm,Wcg,Wcp] = margin(K2*PID2*G2*H2);
% bodeplot(K2*PID2*G2/(1+K2*PID2*G2*H2))

% step response of new closed loop system
% step(K2*PID2*G2/(1+K2*PID2*G2*H2), 4), grid on


