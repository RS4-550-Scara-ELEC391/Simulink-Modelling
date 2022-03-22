%
% Changing Parameters:
%
%

% Load Elec and Mech Data from Simulink:
gearElec = [ ...
    load("out\Kp2Ki0Kd1-Gear10\MotorElec.mat") ...  
    load("out\Kp2Ki0Kd1-Gear25\MotorElec.mat") ...
    load("out\Kp2Ki0Kd1-Gear50\MotorElec.mat") ...
    load("out\Kp2Ki0Kd1-Gear75\MotorElec.mat") ...
    load("out\Kp20Ki0Kd1-Gear75\MotorElec.mat") ...
    load("out\Kp20Ki0Kd1-Gear25\MotorElec.mat") ...
];
gearMech = [ ...
    load('out\Kp2Ki0Kd1-Gear10\MotorMech.mat') ...
    load('out\Kp2Ki0Kd1-Gear25\MotorMech.mat') ...
    load('out\Kp2Ki0Kd1-Gear50\MotorMech.mat') ...
    load('out\Kp2Ki0Kd1-Gear75\MotorMech.mat') ...
    load('out\Kp20Ki0Kd1-Gear75\MotorMech.mat') ...
    load('out\Kp20Ki0Kd1-Gear25\MotorMech.mat') ...
];
output = [ ...
    load("out\Kp2Ki0Kd1-Gear10\output.mat") ...
    load("out\Kp2Ki0Kd1-Gear25\output.mat") ...
    load("out\Kp2Ki0Kd1-Gear50\output.mat") ...
    load("out\Kp2Ki0Kd1-Gear75\output.mat") ...
    load("out\Kp20Ki0Kd1-Gear75\output.mat") ...
    load("out\Kp20Ki0Kd1-Gear25\output.mat") ...
];

start = 4;

figure(1)
clf
hold on
for ii = start:size(output,2)
    plot(gearElec(ii).data.I)
end
lgd = legend('10', '25', '50', '75', '75-Kp20');
title(lgd, 'Gear Ratio');
title('Current v. Time with Varying Gear Ratios')
xlim([0.4 0.75])


figure(2)
clf
hold on
for ii = start:size(output,2)
    plot(output(ii).data.J1_angle)
end
lgd = legend('10', '25', '50', '75');
title(lgd, 'Gear Ratio');
title('J1 Angle v. Time with Varying Gear Ratios')
%xlim([0.4 0.75])

figure(3)
clf
hold on
for ii = start:size(output,2)
plot(gearMech(ii).data.signal2.Motor_Speed)
end
xlim([0.4 0.75])
gd = legend('10', '25', '50', '75');
title(lgd, 'Gear Ratio');
title('Motor Speed v. Time with Varying Gear Ratios')
ylabel('Angular Speed [rad/s]')

