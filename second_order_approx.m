
% % from response graph:
% OS = 2.501              % Peak V - steady-state V
% Ts = 12.20e-3           % Time when V = steady-state V * (1.02)
% Tp = 10.002e-3          % Peak time
% Kdc = 3.894             % Gain = ss V / 5V

% from response graph:
OS = 19.506 - 12.609             % Peak V - steady-state V
Ts = 0.558e-3          % Time when V = steady-state V * (1.02)
Tp = 0.051e-3          % Peak time
Kdc = 12.609


% calculate second-order approx
zeta = sqrt(log(OS/Kdc)^2 / (pi^2 + (log(OS/Kdc))^2))
wn = 4/(Ts*zeta)
beta = pi/(Tp*wn)

tfunc = Kdc*tf([wn^2],[1, 2*zeta*wn, wn^2])
step(tfunc)
pzmap(tfunc)


