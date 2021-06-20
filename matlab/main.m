clc;
clear all;

% INITIALIZE PARAMETERS
% Mechanical parameters
global mw mb Iwx Iwy Ibx Iby Ibz Irx l r R mug mu1 mu2 g m
mw = 0.72; mb = 1.13; % masses
m = mb + mw; % 1.85
l = 0.2438; % length
r = 0.0142; R = 0.102; %radius
% Iwx = mw*(3*R^2+0.03^2)/12; Iwy = 0.5*mw*R^2;
% Ibx = mb*(0.05^2+0.17^2)/12; Iby = mb*(0.1^2+0.17^2)/12; Ibz = mb*(0.1^2+0.05^2)/12;
% Irx = 0.5*0.3*r^2;

Iwx = 0.001925; Iwy = 0.003706;
Ibx = 0.038909; Iby = 0.002608; Ibz = 0.038256;
Irx = 0.5*0.3*r^2;

mug = 0.01; mu1 = 0.005; mu2 = 0.005;
g = 9.81;

% Initial conditions and other settings
fps = 200;
T = 2;
tspan = linspace(0,T,T*fps+1); 
theta1 = 0.5/180*pi; thetadot1 = 0; psidot1 = 0;
theta2 = 0.5/180*pi; thetadot2 = 0; psidot2 = 0;
x0 = [theta1 theta2 thetadot1 thetadot2 psidot1 psidot2];

% CONTROL
% Linear model
global A B
M33 = Iwy + m*R^2;
M44 = Irx + m*r^2;

a1 = Iby + Iwy + mb*l^2 + m*R^2 + 2*mb*l*R;
a2 = Iwy + m*R^2 + mb*l*R;
a3 = - mb*g*l;
a4 = Ibx + Irx + Iwx + mb*l^2 + m*R^2 + 2*mb*l*R;
a5 = Irx + m*R*r + mb*l*r;
a6 = - mb*g*l - m*g*(R - r);

delta1 = a2^2 - a1*M33;
delta2 = a5^2 - a4*M44;

A31 = a3*M33/delta1;
A33 = mug*(M33 - a2)/delta1;
A35 = (mug*M33 - (mug + mu1)*a2)/delta1;

A42 = a6*M44/delta2;
A44 = mug*(M44 - a5)/delta2;
A46 = (mug*M44 - (mug + mu2)*a5)/delta2;

A51 = -a2*a3/delta1; 
A53 = mug*(a1 - a2)/delta1; 
A55 = (-mug*a2 + (mug + mu1)*a1)/delta1;

A62 = -a5*a6/delta2;
A64 = mug*(a4 - a5)/delta2;
A66 = (-mug*a5 + (mug + mu2)*a4)/delta2;

B31 = a2/delta1;
B42 = a5/delta2;
B51 = -a1/delta1;
B62 = -a4/delta2;

A = [0 0 1 0 0 0
     0 0 0 1 0 0
     A31 0 A33 0 A35 0
     0 A42 0 A44 0 A46
     A51 0 A53 0 A55 0
     0 A62 0 A64 0 A66];
B = [0 0
     0 0
     B31 0
     0 B42
     B51 0
     0 B62];
C = [0 0 0 0 1 0
     0 0 0 0 0 1];

global psidot_ref
vx = 0.0;
vy = 0.0; 
psidot_ref = [vx/R;vy/r];
theta_ref = [-mug*psidot_ref(1)/a3;-mug*psidot_ref(2)/a6];

% PI
global Kp1 Ki1 Kp2 Ki2 e1_int e2_int t_last
Kp1 = 0.5*R; Ki1 = 2.2*R;
Kp2 = 0.21*r; Ki2 = 1.2*r;
e1_int = 0; e2_int = 0;
t_last = 0;

% LQR
global K H;
% W1 = diag([50000,10000,500,100]);
% W1 = diag([10000,10000,100,10,1000,100]);
W1 = diag([1,1,1,1,100,10]);
W2 = diag([100,10000]);
K = lqr(A,B,W1,W2);
H = inv(C/(B*K - A)*B);

global theta1_bias theta2_bias
theta1_bias = 1/180*pi;
theta2_bias = -1/180*pi;

% ODE
% global pd1_last pd2_last t_last
% pd1_last = 0;
% pd2_last = 0;
% t_last = -1/fps;
options = odeset('abstol',1e-5,'reltol',1e-5,'Events',@myevent);
[t,x] = ode45(@(t,x) ode_nonlinear(t,x),tspan,x0,options);
% [t,xx] = ode45(@(t,x) ode_linear(t,x),tspan,x0,options);

u = - K*x' + H*psidot_ref;
x(end,1:2)
theta_ref

x(end,5:6)
psidot_ref

% u(:,end)'
% u_ref = [(mug+mu1)*psidot_ref(1);(mug+mu2)*psidot_ref(2)]

subplot(3,1,1)
plot(t,x(:,1:2)*180/pi)
subplot(3,1,2)
plot(t,x(:,5:6)*180/pi)
subplot(3,1,3)
plot(t,u)
% plot((x(:,1:2)-xx(:,1:2))*180/pi)

% xw = R*psi1;
% yr = r*psi2;
% yw = yr + (R - r)*sin(theta2);
% zw = r + (R - r)*cos(theta2);
% xb = xw + l*sin(theta1);
% yb = yw + l*cos(theta1).*sin(theta2);
% zb = zw + l*cos(theta1).*cos(theta2);

% clf
% figure (1)
% animation();
% plot(t,theta1*180/pi,t,theta2*180/pi);
% plot(t,pitchset*180/pi,t,rollset*180/pi);
% plot(t,theta1_bias*180/pi,t,theta2_bias*180/pi);

% for i = 1:length(t)
%     x(i) = trajectory(xi,xf,time,dt,t(i));
% end
% figure (2)
% plot(t,theta1/pi*180,t,theta2/pi*180,'linewidth',1);
% xlabel('Time $t$ [s]','interpreter','latex');
% ylabel('Body Orientation [deg]','interpreter','latex');
% set(gca,'Fontsize',20);
% grid on;
% h1 = legend('$\theta_1$','$\theta_2$');
% set(h1,'interpreter','latex');
% 
% plot(t,R*psi1,t,r*psi2,'linewidth',1);
% xlabel('Time $t$ [s]','interpreter','latex');
% ylabel('Wheel Position [m]','interpreter','latex');
% set(gca,'Fontsize',20);
% grid on;
% h1 = legend('$x_r$','$y_r$');
% set(h1,'interpreter','latex');
% 
% plot(R*psi1,r*psi2,'linewidth',1);
% axis([-1 1 -1 1]*0.02);
% pbaspect([1 1 1])
% xlabel('$x$ [m]','interpreter','latex');
% ylabel('$y$ [m]','interpreter','latex');
% set(gca,'Fontsize',20);
% grid on;
% 
% plot(t,theta1_bias*180/pi,t,theta2_bias*180/pi,'linewidth',1);
% xlabel('Time $t$ [s]','interpreter','latex');
% ylabel('IMU Bias [deg]','interpreter','latex');
% set(gca,'Fontsize',20);
% grid on;
% h1 = legend('$\theta_1$','$\theta_2$');
% set(h1,'interpreter','latex');

function [value,isterminal,direction] = myevent(~,x)
% Locate the time when the value is zero
% increasing direction and don't stop integration.
value = [(abs(x(1)) - pi/2);(abs(x(2)) - pi/2)];   % detect value at zero
isterminal = [1;1]; % don't stop the integration 
direction = [0;0];  % positive direction only
end