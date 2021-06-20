function xdot = ode_nonlinear(t,x)
global mw mb Iwx Iwy Ibx Iby Ibz Irx l r R mug mu1 mu2 g m
global Kp1 Ki1 Kp2 Ki2 e1_int e2_int t_last
global K H psidot_ref
global theta1_bias theta2_bias

% dynamics
t1 = x(1) - theta1_bias;
t2 = x(2) - theta2_bias;
td1 = x(3);
td2 = x(4);
pd1 = x(5);
pd2 = x(6);

e1 = psidot_ref(1) - td1 - pd1;
e2 = psidot_ref(2) - td2 - pd2;
e1_int = e1_int + e1*(t - t_last);
e2_int = e2_int + e2*(t - t_last);

x1 = x(1) - (Kp1*e1 + Ki1*e1_int);
x2 = x(2) - (Kp2*e2 + Ki2*e2_int);
% x1 = t1;
% x2 = t2;
x_lqr = [x1;x2;td1;td2;pd1;pd2];
u = - K*x_lqr + H*psidot_ref;

st1 = sin(t1);
st2 = sin(t2);
ct1 = cos(t1);
ct2 = cos(t2);
d = R + r*(ct2 - 1);

C1 = ((Ibx - Ibz + mb*l^2)*ct1 + mb*l*(R - r))*td2^2*st1 - mb*l*R*td1^2*st1;
C2 = 2*((Ibz - Ibx - mb*l^2)*ct1 - mb*l*d)*td1*td2*st1 - (m*(R - r) + mb*l*ct1)*r*td2^2*st2 - mb*l*r*td1^2*ct1*st2;
C3 = -mb*l*R*td1^2*st1;
C4 = -mb*l*r*td1^2*ct1*ct2 - 2*mb*l*r*td1*td2*st1*st2 - (m*(R - r) + mb*l*ct1)*r*td2^2*st2;
F1 = mug*(td1 + pd1);
F2 = mug*(td2 + pd2);
F3 = mug*(td1 + pd1) + mu1*pd1;
F4 = mug*(td2 + pd2) + mu2*pd2;
G1 = - mb*g*l*st1*ct2;
G2 = -(m*(R - r) + mb*l*ct1)*g*st2;

RHS1 = - C1 - F1 - G1;
RHS2 = - C2 - F2 - G2;
RHS3 = u(1) - C3 - F3;
RHS4 = u(2) - C4 - F4;

M11 = Iby + Iwy + mb*l^2 + (mb + mw)*R^2 + 2*mb*l*R*ct1;
M12 = - mb*l*r*st1*st2;
M13 = Iwy + m*R^2 + mb*l*R*ct1;
M14 = - mb*l*r*st1*st2;
M21 = M12;
M22 = (Ibx + Ibz + mb*l^2)/2 + Irx + Iwx + m*(2*r^2 - 2*r*R + R^2) + 2*m*r*(R - r)*ct2 + (Ibx - Ibz + mb*l^2)*cos(2*t1)/2 + 2*mb*l*ct1*d;
M23 = 0;
M24 = Irx + m*r^2 +r*(m*(R - r) + mb*l*ct1)*ct2;
M31 = M13;
M32 = M23;
M33 = Iwy + m*R^2;
M34 = 0;
M41 = M14;
M42 = M24;
M43 = M34;
M44 = Irx + m*r^2;

M = [M11 M12 M13 M14;M21 M22 M23 M24;M31 M32 M33 M34;M41 M42 M43 M44];
RHS = [RHS1;RHS2;RHS3;RHS4];

t_last = t;
xdot = [td1;td2;M\RHS];