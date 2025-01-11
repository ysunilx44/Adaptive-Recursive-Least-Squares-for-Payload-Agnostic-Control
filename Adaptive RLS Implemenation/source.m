% source.m

% clear all
% clear classes
clc

%ADD HEBI LIBRARY FOR KEYBOARD INPUT
addpath('hebi');

%% May need to change the next line to "kb  = HebiKeyboard();" if you get a keyboard not connected error
kb  = HebiKeyboard('native',1);     %Create a HibiKeyboard object.


%CONSTANTS
mc = 3;                 %Mass of Cart
g  = 9.81;              %Gravity
Mode = 'Acceleration';         %Velocity or Force or Acceleration
min_cycle = 1/60;       %Minimum cycle time
L1  = 0.7;               %Length
m1 = 0.21;                 %Mass of Payload
b1  = 0.0003;               %Damping factor
L2 = 0.7;
m2 = 0.078;
b2 = 0.0003;
a_max = 0.326;
v_max = 0.0726;

%exact frequencies
TH_true = [(L1+L2)*g*(m1+m2)/(L1*L2*m1);
  g^2*(m1+m2)/(L1*L2*m1);
  -1/L1;
  g*(m1+m2)/(L1*L2*m1)]
R_ = m2/m1;
B = sqrt((1+R_)^2*(1/L1+1/L2)^2-4*(1+R_)/(L1*L2));
w_n = sqrt(g/2)*sqrt((1+R_)*(1/L1+1/L2)+[-1;1]*B)

%Setup Filtering
t1_buffer = zeros(1,7);
v_buffer = zeros(1,7);

[num,den] = tfdata(zpk([],[0.7,0.7,0.6,0.5],1,-1));
filter_order = max(length(num{1}),length(den{1}));
Bf = [den{1},zeros(1,filter_order-length(den{1}))];
Af = [num{1},zeros(1,filter_order-length(num{1}))];

t1f_buffer = zeros(1,filter_order);
ddt1f_buffer = zeros(1,filter_order);
ddddt1f_buffer = zeros(1,filter_order);
ddxf_buffer = zeros(1,filter_order);
ddddxf_buffer = zeros(1,filter_order);

%Setup RLS
P = eye(4)*0.1;
L1_est = L1;
m1_est = m1;
L2_est = 0.1;
m2_est = 0.1;
THETA = [(L1_est+L2_est)*g*(m1_est+m2_est)/(L1_est*L2_est*m1_est);
  g^2*(m1_est+m2_est)/(L1_est*L2_est*m1_est);
  -1/L1_est;
  g*(m1_est+m2_est)/(L1_est*L2_est*m1_est)];
forgetting_factor = 1;

%Setup Input Shaping

v_input_buffer = zeros(1,length(0:min_cycle:5));
% T_n = zeros(size(w_n));
% T_n = 2*pi./w_n_final;
% Ai = [0.25 0.25 0.25 0.25];
% ti = [0,T_n(2),T_n(1),T_n(1)+T_n(2)]/2;

%PREPARE THE FIGURE WINDOW & GET PENDULUM OBJECT
figure(1); clf; set(gcf,'KeyPressFcn','1;');

Pend = PENDULUM('InitialStates',[0 0 0 0 0 0], 'WorkspaceLength', 8, ...
    'Damping', b1, 'MassCart', mc, 'MassPendulum', m1, 'PendulumLength', L1, ...
    'SecondPendulumLength', L2, 'SecondPendulumMass', m2, 'SecondPendulumDamping', b2, ...
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

figure(3); clf
subplot(4,1,1);
Stream_TH1 = STREAM_AXIS_DATA(gca);
ylabel('\Theta_1')

subplot(4,1,2);
Stream_TH2 = STREAM_AXIS_DATA(gca);
ylabel('\Theta_2')

subplot(4,1,3);
Stream_wn1 = STREAM_AXIS_DATA(gca);
ylabel('wn_1')

subplot(4,1,4);
Stream_wn2 = STREAM_AXIS_DATA(gca);
ylabel('wn_2')

% setup pre-programmed input and history

% time_quit = 20;
% t_plot = 0:min_cycle:time_quit;
% U_plot = zeros(size(t_plot));
% U_plot(t_plot<4) = 1;
% U_plot(t_plot>=6&t_plot<10) = -1;
% 
% t1_plot = zeros(size(t_plot));
% w1_plot = zeros(size(t_plot));
% w2_plot = zeros(size(t_plot));

%BEGINNING OF SIMULATION LOOP
quit    = 0;
del     = min_cycle;
now = 0;
cycle_idx = 0;
time_quit = 5;
while quit == 0
    cycle_idx = cycle_idx+1;
    %KEEP TRACK OF TIME
    % now = GETTIME;
    now = now+del;
    
    %CHECK IF THE ESC, F1 KEY IS BEING PRESSED
    kbinput = read(kb); 
    if kbinput.ESC 
        quit = 1;
    end
    if now>=time_quit
      quit=1;
    end
    
    %GET VR FROM THE KEYBOARD
    U_manual = GET_MANUAL_COMMAND('LEFT','RIGHT',kb); % Use LEFT and RIGHT arrow keys
    
    %The States
    Q  = Pend.Q;
    t1 = Q(1); % Angle of first pendulum
    t2 = Q(2); % Angle of second pendulum
    w1 = Q(3); % Angular velocity of first pendulum
    w2 = Q(4); % Angular velocity of second pendulum
    x = Q(5);
    v = Q(6);
    c1 = cos(t1);
    s1 = sin(t1);
    c2 = cos(t2);
    s2 = sin(t2);
    c21 = cos(t2-t1);
    s21 = sin(t2-t1);
  
    % Filter inputs to RLS
    T = min_cycle;
    t1_buffer(2:end) = t1_buffer(1:end-1);
    t1_buffer(1) = t1;
    v_buffer(2:end) = v_buffer(1:end-1);
    v_buffer(1) = v;

    
    
    t1f_buffer(2:end) = t1f_buffer(1:end-1);
    Ad = [0 0 0 1 0 0 0];
    Cd = 1;
    t1f_buffer(1) = (Ad*t1_buffer'*Cd-Bf(2:end)*t1f_buffer(2:end)')/Bf(1);

    ddt1f_buffer(2:end) = ddt1f_buffer(1:end-1);
    Ad = [1/90 -3/20 3/2 -49/18 3/2 -3/20 1/90];
    Cd = 1/T^2;
    ddt1f_buffer(1) = (Ad*t1_buffer'*Cd-Bf(2:end)*ddt1f_buffer(2:end)')/Bf(1);

    ddddt1f_buffer(2:end) = ddddt1f_buffer(1:end-1);
    Ad = [-1/6 2 -13/2 28/3 -13/2 2 -1/6];
    Cd = 1/T^4;
    ddddt1f_buffer(1) = (Ad*t1_buffer'*Cd-Bf(2:end)*ddddt1f_buffer(2:end)')/Bf(1);

    ddxf_buffer(2:end) = ddxf_buffer(1:end-1);
    Ad = [-1/60 3/20 -3/4 0 3/4 -3/20 1/60];
    Cd = 1/T;
    ddxf_buffer(1) = (Ad*v_buffer'*Cd-Bf(2:end)*ddxf_buffer(2:end)')/Bf(1);

    ddddxf_buffer(2:end) = ddddxf_buffer(1:end-1);
    Ad = [1/8 -1 13/8 0 -13/8 1 -1/8];
    Cd = 1/T^3;
    ddddxf_buffer(1) = (Ad*v_buffer'*Cd-Bf(2:end)*ddddxf_buffer(2:end)')/Bf(1);

    t1f = Af*t1f_buffer';
    ddt1f = Af*ddt1f_buffer';
    ddddt1f = Af*ddddt1f_buffer';
    ddxf = Af*ddxf_buffer';
    ddddxf = Af*ddddxf_buffer';

    %Solve recursive least squares

    y = ddddt1f;
    PHI = [-ddt1f;-t1f;ddddxf;ddxf];

    % lam_min = 0.7;
    % lam_gain = 0.001;
    % forgetting_factor = lam_min+(1-lam_min)*exp(-lam_gain*(PHI'*diag([1,100,0.01,1])*PHI));

    e = y-THETA'*PHI;
    THETA = THETA+P*PHI*e/(1+PHI'*P*PHI);
    P = (P-P*(PHI*PHI')*P/(1+PHI'*P*PHI))/forgetting_factor;

    wn1 = sqrt(abs(THETA(1)-sqrt(abs(THETA(1)^2-4*THETA(2))))/2);
    wn2 = sqrt(abs(THETA(1)+sqrt(abs(THETA(1)^2-4*THETA(2))))/2);
    
    
    T_n = 2*pi./[wn1,wn2];
    Ai = [0.25 0.25 0.25 0.25];
    ti = [0,T_n(2),T_n(1),T_n(1)+T_n(2)]/2;
    ti_idx = min(round(ti/min_cycle)+1,length(v_input_buffer));

    %Actuate the Pendulum  
    %get input
    v_input = v_max*U_manual;
    % v_input = v_max*U_plot(cycle_idx);
    
    %input shaper
    v_input_buffer(2:end) = v_input_buffer(1:end-1);
    v_input_buffer(1) = v_input;
    v_shaped = sum(Ai.*v_input_buffer(ti_idx));

    %motor input
    a_des = max(min((v_shaped-v)/del,a_max),-a_max);
    Pend.U = a_des;
         
    %Trace states
    Stream_Theta.stream(now, Q(1));
    Stream_Pos.stream(now, Q(5));
    Stream_Vel.stream(now, Q(6));

    Stream_TH1.stream(now, THETA(1));
    Stream_TH2.stream(now, THETA(2));
    Stream_wn1.stream(now, wn1);
    Stream_wn2.stream(now, wn2);

    % t1_plot(cycle_idx) = t1;
    % w1_plot(cycle_idx) = wn1;
    % w2_plot(cycle_idx) = wn2;
     
    %update screen
    drawnow
    
    %HANDLE ANY IDLE TIME
    % finish = GETTIME;    
    % while finish <= now + min_cycle
    %     finish = GETTIME;
    % end
    % del = finish - now;
end
w_n_final = [wn1,wn2];


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
end

%--------------------------------------------------
function out = GETTIME
%time = GETTIME returns the current cpu clock time in milliseconds

tmp = clock;
out = 3600*tmp(4) + 60*tmp(5)+tmp(6);
end
     


