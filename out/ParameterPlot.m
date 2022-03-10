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
];
gearMech = [ ...
    load('out\Kp2Ki0Kd1-Gear10\MotorMech.mat') ...
    load('out\Kp2Ki0Kd1-Gear25\MotorMech.mat') ...
    load('out\Kp2Ki0Kd1-Gear50\MotorMech.mat') ...
    load('out\Kp2Ki0Kd1-Gear75\MotorMech.mat') ...
];
output = [ ...
    load("out\Kp2Ki0Kd1-Gear10\output.mat") ...
    load("out\Kp2Ki0Kd1-Gear25\output.mat") ...
    load("out\Kp2Ki0Kd1-Gear50\output.mat") ...
    load("out\Kp2Ki0Kd1-Gear75\output.mat") ...
];

figure(1)
clf
plot(gearElec(1).data.I)
hold on
plot(gearElec(2).data.I)
plot(gearElec(3).data.I)
plot(gearElec(4).data.I)
lgd = legend('10', '25', '50', '75');
title(lgd, 'Gear Ratio');
title('Current v. Time with Varying Gear Ratios')
xlim([0.4 0.75])


figure(2)
clf
plot(output(1).data.J1_angle)
hold on
plot(output(2).data.J1_angle)
plot(output(3).data.J1_angle)
plot(output(4).data.J1_angle)
lgd = legend('10', '25', '50', '75');
title(lgd, 'Gear Ratio');
title('J1 Angle v. Time with Varying Gear Ratios')
%xlim([0.4 0.75])

figure(3)
clf
hold on
for i = 1:1:4
plot(gearMech(i).data.signal2.Motor_Speed)
end
xlim([0.4 0.75])
gd = legend('10', '25', '50', '75');
title(lgd, 'Gear Ratio');
title('Motor Speed v. Time with Varying Gear Ratios')
ylabel('Angular Speed [rad/s]')

