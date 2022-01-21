% this script generates data to compare 3 differentiation methods:
% simple finite difference: f'(k) = (1/dt)*(f(k)-f(k-1))
% 3-point Savitky-Golay filter: f'(k) = (0.5/dt)*(f(k)-f(k-2))
% homogeneous discrete-time differentiator based on work from
% Proper discretization of homogeneous differentiators, Livne and Levant, 2014

noise_on = 1; % set this value to 1 to add noise to signal, 0 for no noise
noise_mag = 0.01; % maximum magnitude of noise added to signal
dt = 0.02; % dt = t(k)-t(k-1)

t = 0:dt:10;
noise = noise_mag*noise_on*rand(size(t)); % uniformly distributed random noise

f = sin(t)+noise; % known function (sin(t)) plus additive noise
fdot = cos(t); % known derivative function without noise

% since the desired derivative for the homogeneous differentiator is 2nd-order,
% the Lipschitz constant is an upper bound on the absolute value of 
% the third derivative of the input (e.g., abs(-cos(t)) < 2):
L = 2;

% finite difference estimate of derivative:
a = zeros(length(t),1);
a(1) = 1; % assuming that first value of derivative is known
for i=2:length(t)
    a(i) = (f(i)-f(i-1))/dt;
end

% Savitky-Golay filter
b = zeros(length(t),1);
b(1) = 1; b(2) = 1; % assuming that first and second values of derivative are known
for i = 3:length(t)
    b(i) = 0.5*(f(i)-f(i-2))/dt; 
end

% discretized homogeneous differentiator
z0kp1 = 0; z1kp1 = 0; z2kp1 = 0;
z0dot = 1; z1dot = 0; z2dot = -1; % assume the first 3 derivatives are known at t=0
c = zeros(length(t),1);
for i = 1:length(t)
    z0 = z0kp1; z1 = z1kp1; z2 = z2kp1;
    z0dot = -2.12*(L^(1/3))*(abs(z0-f(i))^(2/3))*sign(z0-f(i))+z1;
    z1dot = -2*(L^(2/3))*(abs(z0-f(i))^(1/3))*sign(z0-f(i))+z2;
    z2dot = -1.1*L*sign(z0-f(i));
    
    z0kp1 = z0 + dt*z0dot + 0.5*(dt^2)*z2;
    z1kp1 = z1 + dt*z1dot;
    z2kp1 = z2 + dt*z2dot;
    c(i) = z0dot; 
end
