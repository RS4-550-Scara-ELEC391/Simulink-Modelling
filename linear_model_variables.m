%------------------------------------------------
%   Linear model variables
%------------------------------------------------
close all
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
gear2 = 200;    % --> double check

% Calculated values:
J1 = 2.60;            % from solidworks with 1kg load
J2 = 0.50;

% Amplifier transfer function
        % AmpNum = [2.149e7];
        % AmpDen = [1 655.7 5.52e6];
AmpNum = [1.822e10];
AmpDen = [1 1.434e4 1.445e9];

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




%% Ten Step Process Joint 1

% Step 1: System Identification
Amplifier = tf(AmpNum, AmpDen);

% motor and mech gain
Gm1 = km1*gear1* tf(1,[L1 R1])*tf(1,[J1 B]);
Hm1 = gear1*ka1;
Motormech1 = Gm1 / (1+Gm1*Hm1);

% forward path gain G1
G1 = Amplifier*Motormech1*(1/s);

% feedback path gain H1
H1 = N/(s+N);

% closed loop transfer function, joint 1
cltf1 = G1/(1+G1*H1);

% Step 2: crossover frequency
[Gm,Pm,Wcg,Wcp] = margin(G1*H1); % gain margin, Phase margin, phase crossing freq, gain crossing freq

    % returns the gain margin Gm in absolute units, the phase margin Pm, and 
    % the corresponding frequencies Wcg and Wcp, of sys. Wcg is the frequency 
    % where the gain margin is measured, which is a –180° phase crossing 
    % frequency. Wcp is the frequency where the phase margin is measured, 
    % which is a 0-dB gain crossing frequency


% Step 3 & 4: Add controller dynamics & find X-over freq. (Iterate)
z = Wcg / 10;   % Initial zero 1 decade before PXO
p = 2*CF;
Kp1 = 2/z - 1/p;
Ki1 = 1;
Kd1 = 1/z^2 - Kp1/p;
% for n=1:10
%     PID1 = tf(Kp1 + Ki1/s + Kd1*N*s/(s+N));
%     [Gm,Pm,Wcg,Wcp] = margin(PID1*G1*H1);
%     z = Wcg / 10   % Initial zero 1 decade before PXO
%     p = 2*CF;
%     Kp1 = 2/z - 1/p;
%     Ki1 = 1;
%     Kd1 = 1/z^2 - Kp1/p;
% end

% -> lowest solution z = 8.1446
z = 8.1446;
Kp1 = 2/z - 1/p;
Ki1 = 1;
Kd1 = 1/z^2 - Kp1/p;

PID1 = tf(Kp1 + Ki1/s + Kd1*N*s/(s+N));

% Step 5: Choose initial gain
figure
rlocus(PID1*G1*H1)
K1 = 37.2;

[Gm,Pm,Wcg,Wcp] = margin(K1*PID1*G1*H1)


% Step 6: Nyquist
figure
nyqlog(PID1*G1*H1), grid on, title('Before Gain')
figure
nyqlog(K1*PID1*G1*H1), grid on, title("After Gain, K1 = " + K1)

% Step 7: step response of new closed loop system
figure
step(K1*PID1*G1/(1+K1*PID1*G1*H1)), grid on

% Step 8: Heuristic tuning, evaluate margins
% Step 9: non-linearities, moving target
% Step 10: intended system



%% Ten Step Process Joint 2

% Step 1: System Identification

% motor and mech gain
Gm2 = km2*gear2* tf(1,[L2 R2])*tf(1,[J2 B]);
Hm2 = gear2*ka2;
Motormech2 = Gm2 / (1+Gm2*Hm2);

% forward path gain G1
G2 = Amplifier*Motormech2*(1/s);

% feedback path gain H1
H2 = N/(s+N);

% closed loop transfer function, joint 2
cltf2 = G2/(1+G2*H2);

% Step 2: crossover frequency
[Gm,Pm,Wcg,Wcp] = margin(G2*H2); % gain margin, Phase margin, phase cross freq, gain cross freq

% Step 3 & 4: Add controller dynamics & find X-over freq. (Iterate)
z = Wcg / 10;   % Initial zero 1 decade before PXO
p = 2*CF;
Kp2 = 2/z - 1/p;
Ki2 = 1;
Kd2 = 1/z^2 - Kp2/p;
% for n=1:10
%     PID2 = tf(Kp2 + Ki2/s + Kd2*N*s/(s+N));
%     [Gm,Pm,Wcg,Wcp] = margin(PID2*G2*H2);
%     z = Wcg / 10   % Initial zero 1 decade before PXO
%     p = 2*CF;
%     Kp2 = 2/z - 1/p;
%     Ki2 = 1;
%     Kd2 = 1/z^2 - Kp1/p;
% end

% -> lowest solution z = 13.697
z = 13.697;
Kp2 = 2/z - 1/p;
Ki2 = 1;
Kd2 = 1/z^2 - Kp2/p;

PID2 = tf(Kp2 + Ki2/s + Kd2*N*s/(s+N));

% Step 5: Choose initial gain
figure
rlocus(PID2*G2*H2)
K2 = 100;

[Gm,Pm,Wcg,Wcp] = margin(K2*PID2*G2*H2);

% Step 6: Nyquist
figure
tiledlayout(1,2)
nexttile;
nyqlog(PID2*G2*H2), grid on, title('Before Gain')
nexttile;
nyqlog(K2*PID2*G2*H2), grid on, title("After Gain, K2 = " + K2)

% Step 7: step response of new closed loop system
figure
step(K2*PID2*G2/(1+K2*PID2*G2*H2), 4), grid on

% Step 8: Heuristic tuning, evaluate margins
% Step 9: non-linearities, moving target
% Step 10: intended system
