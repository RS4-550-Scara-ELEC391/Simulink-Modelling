% Derivative filter experiments
close all
t = 0:0.001:8;          % time
u = sin(t);             % input signal
u_noise = awgn(u, 80);  % add AWGN noise (SNR = signal to noise ratio)

% define values
dt = 0.001;     % time between samples
N = 10;         % filtering coeff (choose)
CF = 1000;      % control frequency
a = N*dt / (1+N*dt);    % ratio used in calculations

%initialize
y = zeros(1,length(t));
ydot =  zeros(1,length(t));
y(1) = u_noise(1);
ydot(1) = 0;


% iterate each sample (this is the PID loop)
for i=2:length(t)

    y(i) = ((1-a)*y(i-1) + a*u_noise(i));
    ydot(i) = (a/dt) * (u_noise(i) - y(i-1));

end

% derivative without filtering
deriv_wo_filter = gradient(u_noise)/dt;


%% Plotting

figure
tiledlayout(1,2)
nexttile
plot(t, deriv_wo_filter, t,u_noise)
title('without filtering')
legend derivative input
nexttile
plot(t,ydot,t,u_noise)
title("with filtering, N = "+N)
legend filteredDeriv input
grid on