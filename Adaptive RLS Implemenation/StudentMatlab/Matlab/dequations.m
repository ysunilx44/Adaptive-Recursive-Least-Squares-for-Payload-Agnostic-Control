function qdot = dequations(T,q,u,L,mp,mc,b,Mode)

%Define some parameters that we can use in our differential equations.

mt = mp+mc;
g = 9.81;
t = q(1);
w = q(2);
x = q(3);
v = q(4);
st = sin(t);
ct = cos(t);

switch Mode
    
    case 'Force'
        F = u;
        qdot(1,1) = w;
        qdot(2,1) = -(b*mc*w*ct^2 - F*mp*ct - g*mc*mp*st - g*mp^2*st + b*mc*w*st^2 + b*mp*w*st^2 + L*mp^2*w^2*ct*st)/(L*mp*(mc*ct^2 + mc*st^2 + mp*st^2));
        qdot(3,1) = v;
        qdot(4,1) = (- L*mp*w^2*ct^2*st - L*mp*w^2*st^3 + F*ct^2 + g*mp*ct*st + F*st^2)/(mc*ct^2 + mc*st^2 + mp*st^2); %(- L*mp*w^2*ct^2*st - b*w^2*ct*st^2 - L*mp*w^2*st^3 + b*w*ct*st^2 + F*ct^2 + g*mp*ct*st + F*st^2)/(mc*ct^2 + mc*st^2 + mp*st^2);     

    otherwise %these equations are actually for acceleration control, but the mode
        %for the pendulum class says velocity, and we just differentiate.
        A = u;
        qdot(1,1) = w;
        qdot(2,1) = (- b*w*ct^2 + mp*A*ct - b*w*st^2 + g*mp*st)/(L*mp*(ct^2 + st^2)); % (- b*w^2*st^2 - b*w*ct^2 + mp*A*ct + g*mp*st)/(L*mp*(ct^2 + st^2));
        qdot(3,1) = v;
        qdot(4,1) = A; 
end


