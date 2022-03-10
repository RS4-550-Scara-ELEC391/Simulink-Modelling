%
% Data Analysis:
%   To compare and analyze data we get out from our Simulink/SimX
%   simulations.
%

figure(1)
clf
hold on
title('Position Vs. Time for varying Gear Ratios')
figure(2)
clf
hold on
title('Current Vs. Time for varying Gear Ratios')
figure(3)
clf
hold on
title('Voltage Vs. Time for varying Gear Ratios')



load('mechData_nogear.mat')
load('elecData_nogear.mat')
figure(1)
plot(data.J1_angle)
figure(2)
plot(ans.I)
figure(3)
plot(ans.Vsource)

load('mechData_2-1gear.mat')
load('elecData_2-1gear.mat')
figure(1)
hold on
plot(data.J1_angle )
figure(2)
hold on
plot(ans.I)
figure(3)
hold on
plot(ans.Vsource)


load('mechData_10-1gear.mat')
load('elecData_10-1gear.mat')
figure(1)
hold on
plot(data.J1_angle)
figure(2)
hold on
plot(ans.I)
figure(3)
hold on
plot(ans.Vsource)

load('mechData_1-10gear.mat')
load('elecData_1-10gear.mat')
figure(1)
hold on
plot(data.J1_angle)
figure(2)
hold on
plot(ans.I)
figure(3)
hold on
plot(ans.Vsource)


figure(1)
legend('No Gear', '2:1 Gear', '10:1 Gear', '1:10 Gear')

figure(2)
legend('No Gear', '2:1 Gear', '10:1 Gear', '1:10 Gear')

figure(3)
legend('No Gear', '2:1 Gear', '10:1 Gear', '1:10 Gear')
