clear all;
clc;

syms mw mb R r l Iwx Iwy Ibx Iby Ibz Irx g ug u1 u2 real
syms T1(t) Td1 Tdd1 T2(t) Td2 Tdd2 Phi1(t) Phid1 Phidd1 Phi2(t) Phid2 Phidd2 Psi1(t) Psid1 Psidd1 Psi2(t) Psid2 Psidd2
syms t1 td1 tdd1 t2 td2 tdd2 phi1 phid1 phidd1 phi2 phid2 phidd2 psi1 psid1 psidd1 psi2 psid2 psidd2

Td1 = diff(T1,t); Tdd1 = diff(Td1,t);
Td2 = diff(T2,t); Tdd2 = diff(Td2,t);
Phid1 = diff(Phi1,t); Phidd1 = diff(Phid1,t);
Phid2 = diff(Phi2,t); Phidd2 = diff(Phid2,t);

Psi1 = T1 + Phi1;
Psi2 = T2 + Phi2;
Psid1 = diff(Psi1,t); Psidd1 = diff(Psid1,t);
Psid2 = diff(Psi2,t); Psidd2 = diff(Psid2,t);

Xr = R*Psi1;  Yr = r*Psi2; Zr = r;
Xw = Xr; Yw = Yr + (R - r)*sin(T2); Zw = Zr + (R - r)*cos(T2);
Xb = Xw + l*sin(T1); Yb = Yw + l*cos(T1)*sin(T2); Zb = Zw + l*cos(T1)*cos(T2);

Xwd = diff(Xw,t); Ywd = diff(Yw,t); Zwd = diff(Zw,t);
Xbd = diff(Xb,t); Ybd = diff(Yb,t); Zbd = diff(Zb,t);

T = 1/2*mw*(Xwd^2 + Ywd^2 + Zwd^2) + 1/2*mb*(Xbd^2 + Ybd^2 + Zbd^2) + 1/2*(Iwx*Td2^2 + Iwy*Psid1^2) + 1/2*(Ibx*(Td2*cos(T1))^2 + Iby*Td1^2 + Ibz*(Td2*sin(T1))^2) + 1/2*Irx*Psid2^2;
V = mw*g*Zw + mb*g*Zb;
D = 1/2*ug*(Psid1^2 + Psid2^2) + 1/2*(u1*Phid1^2 + u2*Phid2^2);
La = T - V;

Eqn1 = diff(subs(diff(subs(La,Td1,td1),td1),td1,Td1),t) - diff(subs(La,[T1 Td1],[t1 td1]),t1) + diff(subs(D,Td1,td1),td1);
Eqn2 = diff(subs(diff(subs(La,Td2,td2),td2),td2,Td2),t) - diff(subs(La,[T2 Td2],[t2 td2]),t2) + diff(subs(D,Td2,td2),td2);
Eqn3 = diff(subs(diff(subs(La,Phid1,phid1),phid1),phid1,Phid1),t) - diff(subs(La,[Phi1 Phid1],[phi1 phid1]),phi1) + diff(subs(D,Phid1,phid1),phid1);
Eqn4 = diff(subs(diff(subs(La,Phid2,phid2),phid2),phid2,Phid2),t) - diff(subs(La,[Phi2 Phid2],[phi2 phid2]),phi2) + diff(subs(D,Phid2,phid2),phid2);

eqn1 = simplify(subs(Eqn1,[T1 Td1 Tdd1 T2 Td2 Tdd2 Phi1 Phid1 Phidd1 Phi2 Phid2 Phidd2],[t1 td1 tdd1 t2 td2 tdd2 phi1 phid1 phidd1 phi2 phid2 phidd2]));
eqn2 = simplify(subs(Eqn2,[T1 Td1 Tdd1 T2 Td2 Tdd2 Phi1 Phid1 Phidd1 Phi2 Phid2 Phidd2],[t1 td1 tdd1 t2 td2 tdd2 phi1 phid1 phidd1 phi2 phid2 phidd2]));
eqn3 = simplify(subs(Eqn3,[T1 Td1 Tdd1 T2 Td2 Tdd2 Phi1 Phid1 Phidd1 Phi2 Phid2 Phidd2],[t1 td1 tdd1 t2 td2 tdd2 phi1 phid1 phidd1 phi2 phid2 phidd2]));
eqn4 = simplify(subs(Eqn4,[T1 Td1 Tdd1 T2 Td2 Tdd2 Phi1 Phid1 Phidd1 Phi2 Phid2 Phidd2],[t1 td1 tdd1 t2 td2 tdd2 phi1 phid1 phidd1 phi2 phid2 phidd2]));

RHS1 = simplify(-subs(eqn1,[tdd1 tdd2 phidd1 phidd2],[0 0 0 0]));
RHS2 = simplify(-subs(eqn2,[tdd1 tdd2 phidd1 phidd2],[0 0 0 0]));
RHS3 = simplify(-subs(eqn3,[tdd1 tdd2 phidd1 phidd2],[0 0 0 0]));
RHS4 = simplify(-subs(eqn4,[tdd1 tdd2 phidd1 phidd2],[0 0 0 0]));

M11 = simplify(subs(eqn1,[tdd1 tdd2 phidd1 phidd2],[1 0 0 0]) + RHS1);
M12 = simplify(subs(eqn1,[tdd1 tdd2 phidd1 phidd2],[0 1 0 0]) + RHS1);
M13 = simplify(subs(eqn1,[tdd1 tdd2 phidd1 phidd2],[0 0 1 0]) + RHS1);
M14 = simplify(subs(eqn1,[tdd1 tdd2 phidd1 phidd2],[0 0 0 1]) + RHS1);

M21 = simplify(subs(eqn2,[tdd1 tdd2 phidd1 phidd2],[1 0 0 0]) + RHS2);
M22 = simplify(subs(eqn2,[tdd1 tdd2 phidd1 phidd2],[0 1 0 0]) + RHS2);
M23 = simplify(subs(eqn2,[tdd1 tdd2 phidd1 phidd2],[0 0 1 0]) + RHS2);
M24 = simplify(subs(eqn2,[tdd1 tdd2 phidd1 phidd2],[0 0 0 1]) + RHS2);

M31 = simplify(subs(eqn3,[tdd1 tdd2 phidd1 phidd2],[1 0 0 0]) + RHS3);
M32 = simplify(subs(eqn3,[tdd1 tdd2 phidd1 phidd2],[0 1 0 0]) + RHS3);
M33 = simplify(subs(eqn3,[tdd1 tdd2 phidd1 phidd2],[0 0 1 0]) + RHS3);
M34 = simplify(subs(eqn3,[tdd1 tdd2 phidd1 phidd2],[0 0 0 1]) + RHS3);

M41 = simplify(subs(eqn4,[tdd1 tdd2 phidd1 phidd2],[1 0 0 0]) + RHS4);
M42 = simplify(subs(eqn4,[tdd1 tdd2 phidd1 phidd2],[0 1 0 0]) + RHS4);
M43 = simplify(subs(eqn4,[tdd1 tdd2 phidd1 phidd2],[0 0 1 0]) + RHS4);
M44 = simplify(subs(eqn4,[tdd1 tdd2 phidd1 phidd2],[0 0 0 1]) + RHS4);

M = simplify([M11 M12 M13 M14;M21 M22 M23 M24;M31 M32 M33 M34;M41 M42 M43 M44],100);
RHS = simplify([RHS1;RHS2;RHS3;RHS4],100);