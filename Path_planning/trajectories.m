% --------------------------------------------
%   Generates path with displacement d and 
%   constant acceleration over total time
%   inputs:
%     di = initial position (mm)
%     df = desired final position (mm)
%     time = total time (s)
%     dt = time between points (s)
% --------------------------------------------

function [disp, vel, accel,t] = trajectories(di, df, time, dt)


% default inputs
if nargin==1
  time = 1000;
  dt = 5;
end

d = df - di;
t = linspace(0,time,time/dt); % time in s


% d is total distance (mm) |Od-Oi|

% max velocity at t = time/2
vmax = 2*d/time - 0;

% velocity based on vmax and initial/final v = 0
vel = [(2*vmax/time)*t(1:end/2),(-2*vmax/time)*t(1+ end/2:end)+2*vmax];

% integrate vel to find displacement
disp = cumtrapz(t, vel) ;
disp = disp + di*ones(size(disp));

% differentiate vel to find acceleration
dV = gradient(vel);
accel = dV/dt;


% Plot trajectories
tiledlayout(3,1);

nexttile;
plot(t,disp);
ylabel('d (mm)'), grid on
nexttile;
plot(t,vel);
ylabel('v (mm/s)'), grid on
nexttile;
plot(t,accel);
ylabel('a (mm/s^{2})'), grid on

end