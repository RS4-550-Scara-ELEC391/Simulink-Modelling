%------------------------------------------------
%   draft arm model variables
%------------------------------------------------

% Maxon motor: EC-i 30 30mm (shaft diameter = 4mm)
clear
clc
open_system('draft_arm_model.slx')

% "Make good Choices" - dad:
J1 = 3;
J2 = 3;
loaded = 1;


% Joint 1 Motor:
if (J1 == 1)
    % Maxon motor: EC-i 30 30mm (shaft diameter = 4mm)
    % variables from data sheet:
    V_nom = 24;         % [V]
    R = 0.713;           % [Ohm]
    L = 0.475*1e-3;     % [H]
    km = 22.8*1e-3;     % [Nm/A]
    ka = km;
    motor_name = 'EC-Motor'
    gear_i = 75;

elseif (J1 == 2)
    % Maxon motor: RE-35
    V_nom = 48;     % [V]
    R = 3.04;       % [Ohm]
    L = 0.87;       % [H]
    km = 62.2*1e-3; % [Nm/A]
    ka = km;        % [Nm/A]
    motor_name = 'RE-35'
    gear_i = 75;

elseif (J1 == 3)
    V_nom = 12;
    R = 0.115;
    L = 0.0245e-3; % [H]
    km = 16.4e-3;
    ka = km;
    motor_name = 'RE-40'
    part_number = '148866';
    gear_i = 120;
    %https://www.max 
    % ongroup.com/maxon/view/category/gear?etcc_cu=onsite&etcc_med_onsite=Product&etcc_cmp_onsite=Planetary+Gearheads+(GP)&etcc_plc=Overview-Page-Gears&etcc_var=%5bcom%5d%23en%23_d_&target=filter&filterCategory=planetary
end




% Joint 2 Motor:
if (J2 == 1)
    % Maxon motor: EC-i 30 30mm (shaft diameter = 4mm)
    % variables from data sheet:
    V_nom2 = 24;         % [V]
    R2 = 0.713;           % [Ohm]
    L2 = 0.475*1e-3;     % [H]
    km2 = 22.8*1e-3;     % [Nm/A]
    ka2 = km;
    motor_name2 = 'EC-Motor'
    gear_i2 = 5;
elseif (J2 == 2)
    % Maxon motor: RE-35
    % variables from data sheet:
    V_nom2 = 48;         % [V]
    R2 = 3.04;           % [Ohm]
    L2 = 0.87;     % [H]
    km2 = 62.2*1e-3;     % [Nm/A]
    ka2 = km;
    motor_name2 = 'RE-35'
    gear_i2 = 1;

elseif (J2 == 3)
    % Maxon motor: RE-30
    % variables from data sheet:
    V_nom2 = 12;         % [V]
    R2 = 0.2;           % [Ohm]
    L2 = 0.0345e-3;     % [H]
    km2 = 30.9*1e-3;%13.9*1e-3;     % [Nm/A]
    ka2 = km;
    motor_name2 = 'RE-30'
    part_number2 = '268193';
    gear_i2 = 123;
    gear_number2 = '166170';
end

% Mechanical Variables from Datasheet
Jrot = 13.8*0.0001; % [gm^2]
B = 7e-4;
% calculated values:
Jload = 23.44*0.0001; % calculated from solidworks model
J = Jrot + Jload;   % [gm^2]
J = J/1000;         % [Kgm^2]



% Nicole Tuning:
%{
% P-Controller:
K1 = 39.8;
Kp = K1 * 0.2548;
Ki = K1 * 1;
Kd = K1 * 0.0162;

% P-Controller:
K2 = 1.25;
Kp2 = K2 * 0.1921;
Ki2 = K2 * 1;
Kd2 = K2 * 0.0091;
%}

% No Tuning Test Parameter:
Kp = 10;
Ki = 0;
Kd = 0;

Kp2 = 10;
Ki2 = 0;
Kd2 = 0;


time = 0:0.001:1.999;
path = trajectories(pi, 2, 0.001);
path2 = trajectories(pi, 2, 0.001);


% H-Bridge Amplifier 2nd Order Approximation:
amplifier_num = [2.149e7];
amplifier_denom = [1 655.7 5.52e6];


foldername='VibrationAnalysis/';
% Parameter Plot: Name the Output Files:
%mkdir(strcat('out/',motor_name))
mkdir(strcat('out/', foldername))
%name = sprintf(strcat('out/', motor_name,'/Kp=%03d-Ki=%03d-Kd=%03d-Gear=%03d_data'), Kp, Ki, Kd, gear_i);
name = sprintf(strcat('out/', foldername, motor_name, '-', motor_name2, '-', 'Gear1=%03d-Gear2=%03d-', 'notuning-', 'loaded-%d-badrho', '_data'), gear_i,gear_i2, loaded);


% Set Simulink Model simOut Blocks to write to specific file name:
set_param('draft_arm_model/dataOut','filename', name);
set_param('draft_arm_model/dataOut','MatrixName', 'data')
