function qdot = dequations(T,q,u,L,mp,mc,b,L2, mp2, b2, Mode)

%Define some parameters that we can use in our differential equations.

mt = mp+mc;
g = 9.81;
t1 = q(1);
t2 = q(2);
w1 = q(3);
w2 = q(4);
x = q(5);
v = q(6);
st1 = sin(t1);
ct1 = cos(t1);
st2 = sin(t2);
ct2 = cos(t2);

% delta = t1 - t2;
% sin_delta = sin(delta);
% cos_delta = cos(delta);
% 
% num1 = -g * (2 * mp + mp2) * st1 - mp2 * g * sin(t1-2*t2) ...
%        - 2 * sin_delta * mp2 * (w2^2 * L2 + w1^2 * L * cos_delta);
% den1 = L * (2 * mp + mp2 - mp2 * cos(2 * delta));
% alpha1 = num1 / den1;
% 
% num2 = 2 * sin_delta * (w1^2 * L * (mp + mp2) + g * (mp + mp2) * ct1 ...
%        + w2^2 * L2 * mp2 * cos_delta);
% den2 = L2 * (2 * mp + mp2 - mp2 * cos(2 * delta));
% alpha2 = num2 / den2;

m1 = mp;
m2 = mp2;
L1 = L;
t1_d = w1;
t2_d = w2;
% alpha1 = (m2*cos(t1 - t2)*(- L1*sin(t1 - t2)*t1_d^2 + g*sin(t2) + A*cos(t2)))/(- L1*m2*cos(t1 - t2)^2 + L1*m1 + L1*m2) - (L2*sin(t1 - t2)*t2_d^2 + g*sin(t1)*(m1 + m2) + A*cos(t1)*(m1 + m2))/(- L1*m2*cos(t1 - t2)^2 + L1*m1 + L1*m2);
% alpha2 = (cos(t1 - t2)*(L2*sin(t1 - t2)*t2_d^2 + g*sin(t1)*(m1 + m2) + A*cos(t1)*(m1 + m2)))/(- L2*m2*cos(t1 - t2)^2 + L2*m1 + L2*m2) - ((m1 + m2)*(- L1*sin(t1 - t2)*t1_d^2 + g*sin(t2) + A*cos(t2)))/(- L2*m2*cos(t1 - t2)^2 + L2*m1 + L2*m2);
switch Mode
    
    case 'Force'
        F = u;
        qdot(1,1) = w1; %w1
        qdot(2,1) = w2+ F; %w2
        qdot(3,1) = -alpha1; %α1
        qdot(4,1) = -alpha2; %α2
        qdot(5,1) = 0; %v
        qdot(6,1) = 0; %a

    otherwise %these equations are actually for acceleration control, but the mode
        %for the pendulum class says velocity, and we just differentiate.
        A = u;
        qdot(1,1) = w1; %w1
        qdot(2,1) = w2; %w2
        qdot(3,1) = (m2*cos(t1 - t2)*(- L1*sin(t1 - t2)*t1_d^2 + g*sin(t2) + A*cos(t2)))/(- L1*m2*cos(t1 - t2)^2 + L1*m1 + L1*m2) - (L2*sin(t1 - t2)*t2_d^2 + g*sin(t1)*(m1 + m2) + A*cos(t1)*(m1 + m2))/(- L1*m2*cos(t1 - t2)^2 + L1*m1 + L1*m2); %α1
        qdot(4,1) = (cos(t1 - t2)*(L2*sin(t1 - t2)*t2_d^2 + g*sin(t1)*(m1 + m2) + A*cos(t1)*(m1 + m2)))/(- L2*m2*cos(t1 - t2)^2 + L2*m1 + L2*m2) - ((m1 + m2)*(- L1*sin(t1 - t2)*t1_d^2 + g*sin(t2) + A*cos(t2)))/(- L2*m2*cos(t1 - t2)^2 + L2*m1 + L2*m2); %α2
        qdot(5,1) = v; %v
        qdot(6,1) = A; %A
end


