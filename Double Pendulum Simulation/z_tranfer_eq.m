%% Solving for tf
L1 = 1;
g = 9.81;
L2 = 1.5;
m1 = 1;
m2 = 0.5;
R = m1 / (m1 + m2); % This equals 2/3
T = 0.5; %sampling time 

den = [(L1-R*L1)/g, 0, 1 - L1/L2, 0, -g/L2];
num = [(R-1)/g, 0, R/(2*L2), 0];
sys = tf(num, den);
sys = c2d(sys, T, 'zoh')
%% Changing convention
% Coefficients of numerator and denominator
num = [-0.2417, 0.9461, -0.9461, 0.2417];
den = [1, -3.796, -3.999, -3.796, 1];

% Normalize the transfer function by dividing through by the highest power of z
num_zinv = num / den(1);  % Scale numerator
den_zinv = den / den(1);  % Scale denominator

% Display numerator and denominator with z^-n terms
num_zinv = fliplr(num_zinv); % Flip coefficients for z^-n form
den_zinv = fliplr(den_zinv); % Flip coefficients for z^-n form

% Display the transfer function in z^-n form
disp('Numerator coefficients (z^-n form):');
disp(num_zinv);
disp('Denominator coefficients (z^-n form):');
disp(den_zinv);

% Coefficients for z^-n terms
num_zinv = [0.2417, -0.9461, 0.9461, -0.2417]; % Numerator coefficients for z^-n
den_zinv = [1.0000, -3.7960, -3.9990, -3.7960, 1.0000]; % Denominator coefficients for z^-n

% Define the transfer function using tf
Ts = 0.5; % Sampling time
sys_zinv = tf(num_zinv, den_zinv, Ts, 'Variable', 'z^-1'); % Specify z^-1 as the variable

% Display the transfer function
disp('Transfer function in z^-1 form:');
sys_zinv