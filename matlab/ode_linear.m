function xdot = ode_linear(t,x)
global A B K H psidot_ref
global Kp1 Ki1 Kp2 Ki2 e1_int e2_int t_last
global theta1_bias theta2_bias

t1 = x(1) - theta1_bias;
t2 = x(2) - theta2_bias;

e1 = psidot_ref(1) - x(3) - x(5);
e2 = psidot_ref(2) - x(4) - x(6);
e1_int = e1_int + e1*(t - t_last);
e2_int = e2_int + e2*(t - t_last);

x1 = x(1) - (Kp1*e1 + Ki1*e1_int);
x2 = x(2) - (Kp2*e2 + Ki2*e2_int);
x_lqr = [x1;x2;x(3:end)];
u = - K*x_lqr + H*psidot_ref;
% u1 = max(min(u(1),7.33),-7.33);
% u2 = max(min(u(2),29.32),-29.32);

t_last = t;
xdot = A*[t1;t2;x(2:end)] + B*u;