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
s1 = sin(t1);
c1 = cos(t1);
s2 = sin(t2);
c2 = cos(t2);
s21 = sin(t2-t1);
c21 = cos(t2-t1);

m1 = mp;
m2 = mp2;
L1 = L;

A = u;
qdot(1,1) = w1; %w1
qdot(2,1) = w2; %w2
prob_mat = [L1*c21,L2;
            L1*(m1+m2),L2*m2*c21];
prob_vec = [-A*m2*c2-L1*m2*w1^2*s21-m2*g*s2;
            -A*(m1+m2)*c1+m2*L2*w2^2*s21-(m1+m2)*g*s1];
prob_vec_damp = [-b2*L2*w2-b2*L1*w1*c21-b2*v*c2;
                 -(b+b2)*L1*w1-(b+b2)*v*c1-b2*L2*w2*c21];
qdot(3:4,1) = prob_mat\(prob_vec+prob_vec_damp);
qdot(5,1) = v; %v
qdot(6,1) = A; %a
end

