% --------------------------------------------
%   Generates path with displacement d and 
%   constant acceleration over total time
%   inputs:
%     d = displacement (mm)
%     time = total time (ms)
%     dt = time between points (ms)
% --------------------------------------------

function [disp, vel, accel,t] = trajectories(d, time, dt)


% default inputs
if nargin==1
  time = 1000;
  dt = 5;
end

t = linspace(0,time,time/dt); % time in ms


% d is total distance (mm) |Od-Oi|

% max velocity at t = time/2
vmax = 2*d/time - 0;

% velocity based on vmax and initial/final v = 0
vel = [(2*vmax/time)*t(1:end/2),(-2*vmax/time)*t(1+ end/2:end)+2*vmax];

% integrate vel to find displacement
disp = cumtrapz(t, vel) ;

% differentiate vel to find acceleration
dV = gradient(vel);
accel = dV/dt;


% Plot trajectories
tiledlayout(3,1);

nexttile;
plot(t,disp);
ylabel('d (mm)')
nexttile;
plot(t,vel);
ylabel('v (mm/ms)')
nexttile;
plot(t,accel);
ylabel('a (mm/ms^{s})')

end