syms L1 L2 m1 m2 g x_dd t1 t2 t1_d t2_d t1_dd t2_dd
c21 = cos(t2-t1);
c2 = cos(t2);
s2 = sin(t2);
s21 = sin(t2-t1);
s1 = sin(t1);
c1 = cos(t1);
A = [L1*c21 L2;
    (m1+m2)*L1 m2*L2*c21];
x = [t1_dd; 
    t2_dd];
B = [-g*s2 - x_dd*c2 - L1*t1_d^2*s21;
    -(m1+m2)*g*s1-(m1+m2)*x_dd*c1+L2*t2_d^2*s21];

theta = inv(A)*B
