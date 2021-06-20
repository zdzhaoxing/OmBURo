function A = linearization(x)
global mw mb Iwx Iwy Ibx Iby Ibz Irx l r R mug mu1 mu2 g m

t1 = x(1);
t2 = x(2);
p1 = x(3);
p2 = x(4);
td1 = x(5);
td2 = x(6);
pd1 = x(7);
pd2 = x(8);

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
RHS3 = - C3 - F3;
RHS4 = - C4 - F4;

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
F = [RHS1;RHS2;RHS3;RHS4];

Fq = zeros(4);
Fq(1,1) = mb*g*l;
Fq(2,2) = - (m*(r - R) - mb*l)*g;

Fqd = zeros(4);
Fqd(1,1) = - mug;
Fqd(1,3) = - mug;
Fqd(2,2) = - mug;
Fqd(2,4) = - mug;
Fqd(3,1) = - mug;
Fqd(3,3) = - mu1 - mug;
Fqd(4,2) = - mug;
Fqd(4,4) = - mu2 - mug;

% A = [zeros(4) eye(4) zeros(4,2);zeros(4,2) M\[-Mt1*(M\F)+Fq(:,3) -Mt2*(M\F)+Fq(:,4) Fqd -Ml*(M\F)+Fu(:,1) Fu(:,2)];zeros(1,9) 1;zeros(1,10)];
% B = [zeros(4,1);M\Fu(:,3);0;1];
A = [M\Fq M\Fqd];

end

