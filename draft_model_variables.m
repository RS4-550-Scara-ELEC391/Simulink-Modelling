%------------------------------------------------
%   draft arm model variables
%------------------------------------------------

% Maxon motor: EC-i 30 30mm (shaft diameter = 4mm)
clear
clc
open_system('draft_arm_model.slx')

% "Make good Choices" - dad:
J1 = 1;
J2 = 2;
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
    gear_i2 = 1;
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

end

% Mechanical Variables from Datasheet
Jrot = 13.8*0.0001; % [gm^2]
B = 7e-4;
% calculated values:
Jload = 23.44*0.0001; % calculated from solidworks model
J = Jrot + Jload;   % [gm^2]
J = J/1000;         % [Kgm^2]


% P-Controller:
Kp = 10;
Ki = 0;
Kd = 0;

% P-Controller:
Kp2 = 1;
Ki2 = 0;
Kd2 = 1;


% Parameter Plot: Name the Output Files:
mkdir(strcat('out/',motor_name))
name = sprintf(strcat('out/', motor_name,'/Kp=%03d-Ki=%03d-Kd=%03d-Gear=%03d_data'), Kp, Ki, Kd, gear_i);
% Set Simulink Model simOut Blocks to write to specific file name:
set_param('draft_arm_model/dataOut','filename', name)
set_param('draft_arm_model/dataOut','MatrixName', 'data')
