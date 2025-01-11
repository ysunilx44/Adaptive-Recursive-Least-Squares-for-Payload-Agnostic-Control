function source
clear all
clear classes
clc

%ADD HEBI LIBRARY FOR KEYBOARD INPUT
addpath('hebi');

%% May need to change the next line to "kb  = HebiKeyboard();" if you get a keyboard not connected error
kb  = HebiKeyboard('native',1);     %Create a HibiKeyboard object.


%CONSTANTS
L  = 1.0;               %Length
mc = 3;                 %Mass of Cart
mp = 1;                 %Mass of Payload
b  = 0.1;               %Damping factor
g  = 9.81;              %Gravity
Mode = 'Acceleration';         %Velocity or Force or Acceleration
min_cycle = 0.05;       %Minimum cycle time
L2 = 1.0;
mp2 = 1;
b2 = 0.1;


%PREPARE THE FIGURE WINDOW & GET PENDULUM OBJECT
figure(1); clf; set(gcf,'KeyPressFcn','1;');

Pend = PENDULUM('InitialStates',[0 0 0 0 0 0], 'WorkspaceLength', 8, ...
    'Damping', b, 'MassCart', mc, 'MassPendulum', mp, 'PendulumLength', L, ...
    'SecondPendulumLength', L2, 'SecondPendulumMass', mp2, 'SecondPendulumDamping', b2, ...
    'Mode', Mode);

 
%PREPARE THE STREAMING FIGURE WINDOWS
figure(2); clf

subplot(3,1,1);
Stream_Theta = STREAM_AXIS_DATA(gca);
ylabel('\theta')

subplot(3,1,2);
Stream_Pos = STREAM_AXIS_DATA(gca);
ylabel('Position')

subplot(3,1,3);
Stream_Vel = STREAM_AXIS_DATA(gca);
ylabel('Velocity')


%BEGINNING OF SIMULATION LOOP
quit    = 0;
U_fbl   = 0;
del     = min_cycle;
z1 = 0;
theta_log = [];
%theta = zeros(8, 1);    % Initial parameter vector (assuming 4 y terms and 4 u terms)
theta = [-3.796, -3.999, -3.796, 1, 0.2417, -0.9461, 0.9461, -0.2417]';
P = eye(8);      % Initial covariance matrix
lambda = 1;          % Forgetting factor
n = 4;                  % Number of past terms to consider
m = 4;

% Pre-allocate past inputs/outputs
y_past = zeros(1, n);   % Store past outputs
u_past = zeros(1, m);   % Store past inputs


while quit == 0
    
    %KEEP TRACK OF TIME
    now = GETTIME;
    
    %CHECK IF THE ESC, F1 KEY IS BEING PRESSED
    kbinput = read(kb); 
    if kbinput.ESC 
        quit = 1;
    end
    
    %GET VR FROM THE KEYBOARD
    U_manual = GET_MANUAL_COMMAND('LEFT','RIGHT',kb); % Use LEFT and RIGHT arrow keys
    
    %The States
    Q  = Pend.Q;
    y_current = Q(1); % Example: Use theta as the "output"
    u_current = U_manual; % The current control input

    % Update the regression vector (phi)
    phi = [-y_past, u_past]'; % Regression vector with past outputs and inputs

    % Predict output using current theta
    y_pred = phi' * theta;

    % Compute prediction error
    error = y_current - y_pred;

    % Compute RLS gain
    K = P * phi / (1 + phi' * P * phi);

    % Update theta
    theta = theta + K * error

    % Update covariance matrix
    P = (P - P * phi * phi' * P) / (1 + phi' * P * phi);
    P = P / lambda;

    % Update past inputs and outputs
    y_past = [y_current, y_past(1:end-1)]; % Shift y_past and add new output
    u_past = [u_current, u_past(1:end-1)]; % Shift u_past and add new input
    theta_log = [theta_log; theta'];

%     z1 = 0;
    %Feedback Linearization
    % t = Q(1);
    % w = Q(2);
    % ct = cos(t);
    % st = sin(t);
    t1 = Q(1); % Angle of first pendulum
    t2 = Q(2); % Angle of second pendulum
    w1 = Q(3); % Angular velocity of first pendulum
    w2 = Q(4); % Angular velocity of second pendulum
    ct1 = cos(t1);
    st1 = sin(t1);
    ct2 = cos(t2);
    st2 = sin(t2);
  
    
    %Actuate the Pendulum    
%     Pend.U = U_manual; 
%     Pend.U = U_fbl;
    Pend.U = U_manual;
         
    %Trace states
    Stream_Theta.stream(now, Q(1));
    Stream_Pos.stream(now, Q(5));
    Stream_Vel.stream(now, Q(6));
     
    %update screen
    drawnow
    
    %HANDLE ANY IDLE TIME
    finish = GETTIME;    
    while finish <= now + min_cycle;
        finish = GETTIME;
    end
    del = finish - now;
end

figure;
plot(theta_log);
%title('Evolution of Parameters');
xlabel('Time Steps');
ylabel('Parameter Values');
legend('a', 'b', 'c', 'd', 'e', 'f', 'g', 'h')
%print(theta_log)
disp(std(theta_log, 0, 1))
%--------------------------------------------------
function Uout = GET_MANUAL_COMMAND(xm, xp, kb)
umax 	= 1;
U       = 0; 
kbinput = read(kb);

if isfield(kbinput,xm) && isfield(kbinput,xp) % if using meta keys such as arrows
    if kbinput.(xp) % move right with specified meta key (e.g. RIGHT arrow)
        U = U + umax;
    end
    if kbinput.(xm) % move left with specified key (e.g. LEFT arrow)
        U = U - umax; 
    end
else % using regular keys
    if sum(kbinput.keys) ~= 0
        if kbinput.keys(xp)% move right with specified key (e.g. 'd')
            U = U + umax;
        end
        if kbinput.keys(xm) % move left with specified key (e.g. 'a')
            U = U - umax; 
        end
    end
end

Uout = U;

%--------------------------------------------------
function out = GETTIME
%time = GETTIME returns the current cpu clock time in milliseconds

tmp = clock;
out = 3600*tmp(4) + 60*tmp(5)+tmp(6);
     


