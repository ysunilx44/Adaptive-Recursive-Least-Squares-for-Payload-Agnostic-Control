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
Mode = 'Force';         %Velocity or Force or Acceleration
min_cycle = 0.05;       %Minimum cycle time

%PREPARE THE FIGURE WINDOW & GET PENDULUM OBJECT
figure(1); clf; set(gcf,'KeyPressFcn','1;');

Pend = PENDULUM('InitialStates',[pi 0 0 0], 'WorkspaceLength', 8,'Damping',b,...
    'MassCart', mc, 'MassPendulum', mp, 'PendulumLength',L,'Mode',Mode);

 
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
    
    %Feedback Linearization   
    switch Mode
        case 'Force'   
                %U_fbl = ??
      
        case 'Velocity'
                %U_fbl = ??
            
         case 'Acceleration'
                %U_fbl = ??
    end
    
    %Full State Feedback (from LQR)
    switch Mode
        case 'Force'   
                %U_lqr = ??
      
        case 'Velocity'
                %U_lqr = ??
                
                %hint: this mode is sensitive to numerical error that manifests itself in the form of 'integral wind-up'. See if
                %you can understand why this is the case. To make this mode work reliably in this simulation environment, you need to 
                %mitigate the integral wind-up.
                
                %K       = [??  ??  ??];
                %z1      = z1*0.99 + Q(1)*del;               %0.99 mitigates integral wind-up in the z1 state caused by numerical errors.
                %z2      = Q(1);
                %z3      = Q(3);
                %Z       = [z1 z2 z3]';
                %U_lqr   = -K * Z;
            
         case 'Acceleration'
                %U_lqr = ??
    end
  
    
    %Actuate the Pendulum    
    Pend.U = U_manual;     
         
    %Trace states
    Stream_Theta.stream(now, Q(1));
    Stream_Pos.stream(now, Q(3));
    Stream_Vel.stream(now, Q(4));
     
    %update screen
    drawnow
    
    %HANDLE ANY IDLE TIME
    finish = GETTIME;    
    while finish <= now + min_cycle;
        finish = GETTIME;
    end
    del = finish - now;
end


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
     


