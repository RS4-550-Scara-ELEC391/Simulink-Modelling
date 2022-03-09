%
% Data Analysis:
%   To compare and analyze data we get out from our Simulink/SimX
%   simulations.
%

load('mechData.mat')
load('elecData.mat')

figure(1)
clf
plot(mechData.J1_angle)

figure(2)
clf
plot(elecData.I)
