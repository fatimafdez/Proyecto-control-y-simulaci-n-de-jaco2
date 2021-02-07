% Funciones de transferencia de cada eslabon
clc;clear;

% Se carga el fichero con las matrice Ma, Va, Ga, y se realiza las
% sustituciones necesarias para poder obtener las matrices aproximadas, asi
% como las matrices finales. Se calculan también las funciones de
% transferencia de cada eslabón del robot.

% LAS SUSTITUCIONES NECESARIAS SON PARA ESTE CASO EXCLUSIVAMENTE, POR TANTO
% REVISAR SIEMPRE LAS SUSTITUCIONES, CAMBIARLAS MANUALMENTE
clearvars;clc
load('ma_va_ga_LAG.mat');
syms q1 q2 q3 q4 q5 q6 qd1 qd2 qd3 qd4 qd5 qd6 qdd1 qdd2 qdd3 qdd4 qdd5 qdd6

d1=simplify(subs(diag(Ma),[q1 q2 q3 q4 q5 q6],[0 0 0 0 0 0]));   

% Eslabon 1
d21=simplify(subs(Va(1),[qd1 qd2 qd3 qd4 qd5 qd6 qdd1 qdd2 qdd3 qdd4 qdd5 qdd6],[1 0 0 0 0 0 0 0 0 0 0 0]));
G1=tf(1,[double(d1(1)) double(d21) 0])

% Eslabon 2
d22=simplify(subs(Va(2),[qd1 qd2 qd3 qd4 qd5 qd6 qdd1 qdd2 qdd3 qdd4 qdd5 qdd6],[0 1 0 0 0 0 0 0 0 0 0 0]));
G2=tf(1,[double(d1(2)) double(d22) 0])

% Eslabon 3
d23=simplify(subs(Va(3),[qd1 qd2 qd3 qd4 qd5 qd6 qdd1 qdd2 qdd3 qdd4 qdd5 qdd6],[0 0 1 0 0 0 0 0 0 0 0 0]));
G3=tf(1,[double(d1(3)) double(d23) 0])

% Eslabon 4
d24=simplify(subs(Va(4),[qd1 qd2 qd3 qd4 qd5 qd6 qdd1 qdd2 qdd3 qdd4 qdd5 qdd6],[0 0 0 1 0 0 0 0 0 0 0 0]));
G4=tf(1,[double(d1(4)) double(d24) 0])

% Eslabon 5
d25=simplify(subs(Va(5),[qd1 qd2 qd3 qd4 qd5 qd6 qdd1 qdd2 qdd3 qdd4 qdd5 qdd6],[0 0 0 0 1 0 0 0 0 0 0 0]));
G5=tf(1,[double(d1(5)) double(d25) 0])

% Eslabon 6
d26=simplify(subs(Va(6),[qd1 qd2 qd3 qd4 qd5 qd6 qdd1 qdd2 qdd3 qdd4 qdd5 qdd6],[0 0 0 0 0 1 0 0 0 0 0 0]));
G6=tf(1,[double(d1(6)) double(d26) 0])

Ma_aprox = double(diag(d1))
Va_aprox = double([d21;d22;d23;d24;d25;d26])

% clearvars -except G1 G2 G3 G4 G5 G6 Ma_aprox Va_aprox
% save('JACO_2_funcionesTransferencia.mat');

