% Derivative filter experiments

t = 0:0.001:8;          % time
u = sin(t);             % input signal
u_noisy = awgn(u, 80);  % add AWGN noise (SNR = signal to noise ratio = 80)

% define values
dt = 0.001;     % time between samples (1/CF)
N = 10;         % filtering coeff (choose)
CF = 1000;      % control frequency
a = N*dt / (1+N*dt);    % ratio used in calculations

%initialize
y = zeros(1,length(t));         % y(1) = 0;     
ydot =  zeros(1,length(t));     % ydot(1) = 0;

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Note: 'y' is the output of the filter and 'ydot' is the output of the
% derivative. 'ydot' will have to be multiplied by Kd but that's not part
% of this function.
%          u(t)            y(t)     ydot(t)   
%     error ----> N/(s+N) ------> s -------> Kd ----> add to PID terms
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


% iterate each sample (this is for the PID loop)
for i=2:length(t)

    y(i) = ((1-a)*y(i-1) + a*u_noisy(i));
    ydot(i) = (a/dt) * (u_noisy(i) - y(i-1));

end

% derivative without filtering (just for comparison)
deriv_wo_filter = gradient(u_noisy)/dt;


%% Plotting

figure(5)
tiledlayout(1,2)
nexttile
plot(t, deriv_wo_filter, t,u_noisy)
title('without filtering')
legend derivative input
nexttile
plot(t,ydot,t,u_noisy)
title("with filtering, N = "+N)
legend filteredDeriv input
grid on