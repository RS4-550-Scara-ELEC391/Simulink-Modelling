%
% Parameter Plot v2
%
%

% Get data, each element is the struct containing the with its' name.
clear
clc
%close all
  
datapath = 'out/VibrationAnalysis/'
%datapath = 'out/RE-35/'
A = dir(strcat(datapath,'*.mat'));
A = {A.name};
n = size(A,2);

for ii = 1:n
    data{ii} = load(strcat(datapath,A{ii}));
    names{ii} = A{ii};
end



% Plot Data:
figure(1)
clf
tl = tiledlayout(1,2);
tl.TileSpacing = 'tight';
tl.Padding = 'compact';

nexttile
hold on
for ii = 1:n
    p = plot(data{ii}.data.Motor1Elec.I,'DisplayName',names{ii}(1:end-9));
    p.LineWidth = 1.5;
end
xlim([0.4 0.75])
legend('Location','southeast');
title('Current v. Time with Varying Gear Ratios')
grid on; grid minor
hold off

nexttile
hold on
for ii = 1:n
    p=plot(data{ii}.data.RobotOutput.J1_angle,'DisplayName',names{ii}(1:end-9));
    p.LineWidth = 1.5;
end
xlim([0.4 2])
legend('Location','southeast')
title('J1 Angle v. Time with Varying Gear Ratios')
grid on; grid minor



figure(2)
clf
tl = tiledlayout(1,1);
tl.TileSpacing = 'tight';
tl.Padding = 'compact';

nexttile
hold on
for ii = 1:n
    p = plot(data{ii}.data.MotorMech.signal2.Motor_Torque,'DisplayName',names{ii}(1:end-9));
    p.LineWidth = 1.5;
end
xlim([0.4 0.75])
legend('Location','southeast');
title('Motor Torque v. Time with Varying Gear Ratios')
grid on; grid minor
hold off



figure(3)
clf
tl = tiledlayout(1,1);
tl.TileSpacing = 'tight';
tl.Padding = 'compact';

nexttile
hold on
for ii = 1:n
    %power_in{ii} = data{ii}.data.Motor1Elec.I * data{ii}.data.Motor1Elec.signal1;
    power_in{ii} = data{ii}.data.Motor1Elec.I * data{ii}.data.Motor1Elec.Vsource;
    p = plot(power_in{ii}, 'DisplayName',names{ii}(1:end-9));
    p.LineWidth = 1.5;
end
xlim([0.4 1.2])
legend('Location','southeast');
title('Electrical Power in v. Time with Varying Gear Ratios')
grid on; grid minor
hold off

figure(4)
clf
hold on
for ii = 1:n
    p=plot(data{1}.data.RobotOutput.J1_speed) 
    p.LineWidth = 1.5;
end
%xlim([0.4 1.2])
legend('Location','southeast');
title('J1Speed in v. Time with Varying Gear Ratios')
grid on; grid minor
hold off



figure(5)
clf
hold on
for ii = 1:n
    %p=plot(data{1}.data.Motor1Elec.signal1) 
    p=plot(data{1}.data.Motor1Elec.Vsource) 
    p.LineWidth = 1.5;
end
%xlim([0.4 1.2])
legend('Location','southeast');
title('Voltage Source J1 in v. Time with Varying Gear Ratios')
grid on; grid minor
hold off
