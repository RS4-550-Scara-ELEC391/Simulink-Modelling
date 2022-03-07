% ---------------------------------------------------
%   Outputs two pairs of theta1, theta2 for the 
%   corresponding end effector coordinates
%   intputs: L1 = link1 length, L2 = link2 length
%   (XE, YE) = x,y coordinates of end-effector
% ---------------------------------------------------


function [theta1_1, theta2_1, theta1_2, theta2_2] = xy2theta(L1,L2,XE,YE)


theta1_1 = 2*atand((2*L1*YE + (- L1^4 + 2*L1^2*L2^2 + 2*L1^2*XE^2 + 2*L1^2*YE^2 - L2^4 + 2*L2^2*XE^2 + 2*L2^2*YE^2 - XE^4 - 2*XE^2*YE^2 - YE^4)^(1/2))/(L1^2 + 2*L1*XE - L2^2 + XE^2 + YE^2));
theta1_2 = 2*atand((2*L1*YE - (- L1^4 + 2*L1^2*L2^2 + 2*L1^2*XE^2 + 2*L1^2*YE^2 - L2^4 + 2*L2^2*XE^2 + 2*L2^2*YE^2 - XE^4 - 2*XE^2*YE^2 - YE^4)^(1/2))/(L1^2 + 2*L1*XE - L2^2 + XE^2 + YE^2));


theta2_1 = -2*atand(((- L1^2 + 2*L1*L2 - L2^2 + XE^2 + YE^2)*(L1^2 + 2*L1*L2 + L2^2 - XE^2 - YE^2))^(1/2)/(- L1^2 + 2*L1*L2 - L2^2 + XE^2 + YE^2));
theta2_2 =  2*atand(((- L1^2 + 2*L1*L2 - L2^2 + XE^2 + YE^2)*(L1^2 + 2*L1*L2 + L2^2 - XE^2 - YE^2))^(1/2)/(- L1^2 + 2*L1*L2 - L2^2 + XE^2 + YE^2));


end