% Matriz de Denavit-Hartenberg -->  Notacion Standard
% clear all
format compact
syms teta d a alfa real;
Aij=MDH(teta,d,a,alfa);
%Aij=trotz(teta)*transl(0,0,d)*transl(a,0,0)*trotx(alfa)
pi1=sym('pi');

%   para obtención del Modelo Cinemático Directo estandar  
syms q1 q2 q3 q4 q5 q6 D1 D2 D3 D4 D5 D6 e2 d4b d5b d6b shi aa theta phi real;
pi1=sym('pi');

%% Parámetros de Denavit - Hartenberg
A01 = MDH(-q1,       D1,         0,         pi/2);
A12 = MDH(q2+pi/2,   0,          D2,        pi);
A23 = MDH(q3-pi/2,   -e2,        0,         pi/2);
A34 = MDH(q4,        -d4b,       0,         pi/3);
A45 = MDH(q5+pi,     -d5b,       0,         pi/3);
A56 = MDH(q6-pi/2,   -d6b,       0,         pi);

% lo he llamado phi al angulo para que salga mas bonito
T = A01*A12*A23*A34*A45*A56;
px1=T(1,4);
py1=T(2,4);
pz1=T(3,4);

%% Variables auxiliares - DATASHEET PAG 8 - 11
D1v = 0.2755;    D2v = 0.4100;    D3v = 0.2073;    D4v = 0.0741;
D5v = 0.0741;    D6v = 0.1600;    e2v = 0.0098;

aa = pi/6;
ca = cos(aa);       sa = sin(aa);
c2a = cos(2*aa);    s2a = sin(2*aa);

d4bv = D3 + sa/s2a * D4;
d5bv = sa/s2a * D4 + sa/s2a * D5;
d6bv = sa/s2a * D5 + D6;

%% Sustitución
T_sus = subs(T,[D1 D2 D3 D4 D5 D6 e2 d4b d5b d6b],[D1v D2v D3v D4v D5v D6v e2v d4bv d5bv d6bv])