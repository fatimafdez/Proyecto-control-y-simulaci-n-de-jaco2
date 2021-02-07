% Ejemplo de la utilizaci�n del algoritmo de Lagrange para la din�mica
% de un robot de 3 DGL
% M.G. Ortega (2020)

param_inercias;

% Elegir entre R (rotaci�n) y P (prism�tica)
Tipo_Q1 = 'R';
Tipo_Q2 = 'R';
Tipo_Q3 = 'R';
Tipo_Q4 = 'R';
Tipo_Q5 = 'R';
Tipo_Q6 = 'R';

if ( (Tipo_Q1 ~= 'R') & (Tipo_Q1 ~='P')); error('Elegir R o P para Tipo_Q1'); end;
if ( (Tipo_Q2 ~= 'R') & (Tipo_Q2 ~='P')); error('Elegir R o P para Tipo_Q2'); end;
if ( (Tipo_Q3 ~= 'R') & (Tipo_Q3 ~='P')); error('Elegir R o P para Tipo_Q3'); end;
if ( (Tipo_Q4 ~= 'R') & (Tipo_Q4 ~='P')); error('Elegir R o P para Tipo_Q1'); end;
if ( (Tipo_Q5 ~= 'R') & (Tipo_Q5 ~='P')); error('Elegir R o P para Tipo_Q2'); end;
if ( (Tipo_Q6 ~= 'R') & (Tipo_Q6 ~='P')); error('Elegir R o P para Tipo_Q3'); end;



% Definici�n de variables simb�licas
syms T1 T2 T3 q1 qd1 qdd1 q2 qd2 qdd2 q3 qd3 qdd3 q4 qd4 qdd4 q5 qd5 qdd5 q6 qd6 qdd6 g real
% pi = sym('pi'); % Importante para c�culo simb�lico

% DATOS CINEM�TICOS DEL BRAZO DEL ROBOT
% Dimensiones (m)
D1 = 0.2755;    D2 = 0.4100;    D3 = 0.2073;    D4 = 0.0741;
D5 = 0.0741;    D6 = 0.1600;    e2 = 0.0098;

aa = pi/6;

ca = cos(aa);       sa = sin(aa);
c2a = cos(2*aa);    s2a = sin(2*aa);

d4b = D3 + sa/s2a * D4;
d5b = sa/s2a * D4 + sa/s2a * D5;
d6b = sa/s2a * D5 + D6;

% Par�metros de Denavit-Hartenberg (utilizado en primera regla de Newton-Euler)
% Eslab�n 1:
theta1 = -q1;           d1 = D1;        a1 = 0;         alpha1 = pi/2;
% Eslab�n 2:
theta2 = q2 + pi/2;     d2 = 0;      	a2 = D2;        alpha2 = pi;
% Eslab�n 3:
theta3 = q3 - pi/2;     d3 = -e2;      	a3 = 0;         alpha3 = pi/2;
% Eslab�n 4
theta4 = q4;            d4 = -d4b;      a4 = 0;         alpha4 = pi/3;
% Eslab�n 5
theta5 = q5 + pi;     	d5 = -d5b;      a5 = 0;         alpha5 = pi/3;
% Eslab�n 6
theta6 = q6 - pi/2;   	d6 = -d6b;     	a6 = 0;         alpha6 = pi;

% DATOS DIN�MICOS DEL BRAZO DEL ROBOT
% Ejecutar el fichero de inercias

% Eslab�n 1
m1 = masa(1); % kg
s11 = cdm(1,:)'; % m
I11 = matrices_inercia(1:3,:); % kg.m2

% Eslab�n 2
m2 = masa(2); % kg
s22 = cdm(2,:)'; % m
I22 = matrices_inercia(4:6,:); % kg.m2

% Eslab�n 3
m3 = masa(3); % kg
s33 = cdm(3,:)'; % m
I33 = matrices_inercia(7:9,:); % kg.m2

% Eslab�n 4
m4 = masa(4); % kg
s44 = cdm(4,:)'; % m
I44 = matrices_inercia(10:12,:); % kg.m2

% Eslab�n 5
m5 = masa(5); % kg
s55 = cdm(5,:)'; % m
I55 = matrices_inercia(13:15,:); % kg.m2

% Eslab�n 6
m6 = masa(6); % kg
s66 = cdm(6,:)'; % m
I66 = matrices_inercia(16:18,:); % kg.m2


% DATOS DE LOS MOTORES
% Inercias
Jm1 = min(diag(I11)); Jm2 = min(diag(I22)); Jm3 = min(diag(I33)); % kg.m2
Jm4 = min(diag(I44)); Jm5 = min(diag(I55)); Jm6 = min(diag(I66)); % kg.m2

% Coeficientes de fricci�n viscosa
Bm1 = 3.6e-5; Bm2 = 3.6e-5; Bm3 = 3.6e-5; % N.m / (rad/s)
Bm4 = 3.6e-5; Bm5 = 3.6e-5; Bm6 = 3.6e-5; % N.m / (rad/s)
% Factores de reducci�n
R1 = 1; R2 = 1; R3 = 1;
R4 = 1; R5 = 1; R6 = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ALGOR�TMO DE C�LCULO DE LA CINEM�TICA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wij : velocidad angular absoluta de eje j expresada en i
% wdij : aceleraci�n angular absoluta de eje j expresada en i
% vij : velocidad lineal absoluta del origen del marco j expresada en i
% vdij : aceleraci�n lineal absoluta del origen del marco j expresada en i
% aii : aceleraci�n del centro de gravedad del eslab�n i, expresado en i?

% fij : fuerza ejercida sobre la articulaci�n j-1 (uni�n barra j-1 con j),
% expresada en i-1
%
% nij : par ejercido sobre la articulaci�n j-1 (uni�n barra j-1 con j),
% expresada en i-1

% pii : vector (libre) que une el origen de coordenadas de i-1 con el de i,
% expresadas en i : [ai, di*sin(alphai), di*cos(alphai)] (a,d,aplha: par�metros de DH)
%
% sii : coordenadas del centro de masas del eslab�n i, expresada en el sistema
% i

% Iii : matriz de inercia del eslab�n i expresado en un sistema paralelo al
% i y con el origen en el centro de masas del eslab�n
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Condiciones iniciales de la base
w00=[0 0 0]';
v00 = [0 0 0]';

% Eje Z local
Z=[0 0 1]';

% Eslab�n 1
p11 = [a1, d1*sin(alpha1), d1*cos(alpha1)]';
% Eslab�n 2
p22 = [a2, d2*sin(alpha2), d2*cos(alpha2)]';
% Eslab�n 3
p33 = [a3, d3*sin(alpha3), d3*cos(alpha3)]';
% Eslab�n 4
p44 = [a4, d4*sin(alpha4), d4*cos(alpha4)]';
% Eslab�n 5
p55 = [a5, d5*sin(alpha2), d5*cos(alpha5)]';
% Eslab�n 6
p66 = [a6, d6*sin(alpha6), d6*cos(alpha6)]';


% Obtenci�n de las matrices de rotaci�n (i)R(i-1) y de sus inversas
R01=[cos(theta1) -cos(alpha1)*sin(theta1) sin(alpha1)*sin(theta1);
    sin(theta1)  cos(alpha1)*cos(theta1)  -sin(alpha1)*cos(theta1);
    0            sin(alpha1)              cos(alpha1)           ];
R10= R01';

R12=[cos(theta2) -cos(alpha2)*sin(theta2) sin(alpha2)*sin(theta2);
    sin(theta2)  cos(alpha2)*cos(theta2)  -sin(alpha2)*cos(theta2);
    0            sin(alpha2)              cos(alpha2)           ];
R21= R12';

R23=[cos(theta3) -cos(alpha3)*sin(theta3) sin(alpha3)*sin(theta3);
    sin(theta3)  cos(alpha3)*cos(theta3)  -sin(alpha3)*cos(theta3);
    0            sin(alpha3)              cos(alpha3)           ];
R32= R23';

R34=[cos(theta4) -cos(alpha4)*sin(theta4) sin(alpha4)*sin(theta4);
    sin(theta4)  cos(alpha4)*cos(theta4)  -sin(alpha4)*cos(theta4);
    0            sin(alpha4)              cos(alpha4)           ];
R43= R34';

R45=[cos(theta5) -cos(alpha5)*sin(theta5) sin(alpha5)*sin(theta5);
    sin(theta5)  cos(alpha5)*cos(theta5)  -sin(alpha5)*cos(theta5);
    0            sin(alpha5)              cos(alpha5)           ];
R54= R45';

R56=[cos(theta6) -cos(alpha6)*sin(theta6) sin(alpha6)*sin(theta6);
    sin(theta6)  cos(alpha6)*cos(theta6)  -sin(alpha6)*cos(theta6);
    0            sin(alpha6)              cos(alpha6)           ];
R65= R56';
% C�LCULO DE LA LAGRANGIANA
% L = K - U

% ENERG�A CIN�TICA
% K = sum_i=1_N Ki
%  donde
%         Ki = 1/2 mi * vcii'*vcii + 1/2* wii' * Ici*wii

% C�lculo de las velocidades angulares
% Articulaci�n 1
if (Tipo_Q1=='R');
    w11= R10*(w00+Z*qd1);  % Si es de rotaci�n
else
    w11 = R10*w00;      % Si es de translaci�n
end
% Articulaci�n 2
if (Tipo_Q2=='R');
    w22= R21*(w11+Z*qd2);  % Si es de rotaci�n
else
    w22 = R21*w11;      % Si es de translaci�n
end
% Articulaci�n 3
if (Tipo_Q3=='R');
    w33= R32*(w22+Z*qd3);  % Si es de rotaci�n
else
    w33 = R32*w22;      % Si es de translaci�n
end
% Articulaci�n 4
if (Tipo_Q1=='R');
    w44= R43*(w33+Z*qd4);  % Si es de rotaci�n
else
    w44 = R43*w33;      % Si es de translaci�n
end
% Articulaci�n 5
if (Tipo_Q2=='R');
    w55= R54*(w44+Z*qd5);  % Si es de rotaci�n
else
    w55 = R54*w44;      % Si es de translaci�n
end
% Articulaci�n 6
if (Tipo_Q3=='R');
    w66= R65*(w55+Z*qd6);  % Si es de rotaci�n
else
    w66 = R65*w55;      % Si es de translaci�n
end


% C�lculo de las velocidades lineales del origen de los marcos
% Articulaci�n 1
if (Tipo_Q1=='R');
    v11 = R10*v00+cross(w11,p11); % Si es de rotaci�n
else
    v11 = R10*(v00 + qd1*Z) +cross(w11,p11) ; % Si es de translaci�n
end
% Articulaci�n 2
if (Tipo_Q2=='R');
    v22 = R21*v11+cross(w22,p22); % Si es de rotaci�n
else
    v22 = R21*(v11+ qd2*Z)+cross(w22,p22); % Si es de translaci�n
end
% Articulaci�n 3
if (Tipo_Q3=='R');
    v33 = R32*v22+cross(w33,p33); % Si es de rotaci�n
else
    v33 = R32*(v22+qd3*Z)+cross(w33,p33); % Si es de translaci�n
end
% Articulaci�n 4
if (Tipo_Q4=='R');
    v44 = R43*v33+cross(w44,p44); % Si es de rotaci�n
else
    v44 = R43*(v33+qd4*Z)+cross(w44,p44); % Si es de translaci�n
end
% Articulaci�n 5
if (Tipo_Q5=='R');
    v55 = R54*v44+cross(w55,p55); % Si es de rotaci�n
else
    v55 = R54*(v44+qd5*Z)+cross(w55,p55); % Si es de translaci�n
end
% Articulaci�n 6
if (Tipo_Q6=='R');
    v66 = R65*v55+cross(w66,p66); % Si es de rotaci�n
else
    v66 = R65*(v55+qd6*Z)+cross(w66,p66); % Si es de translaci�n
end

% C�lculo de las velocidades lineales de los centros de masas a partir de la velocidad lineal del origen del marco.
% Articulaci�n 1
vc11 = v11+cross(w11,s11); % Suponiendo que el centro de gravedad se mueva a la misma velocidad que el eslab�n
% Articulaci�n 2
vc22 = v22+cross(w22,s22); % Suponiendo que el centro de gravedad se mueva a la misma velocidad que el eslab�n
% Articulaci�n 3
vc33 = v33+cross(w33,s33); % Suponiendo que el centro de gravedad se mueva a la misma velocidad que el eslab�n
% Articulaci�n 4
vc44 = v44+cross(w44,s44); % Suponiendo que el centro de gravedad se mueva a la misma velocidad que el eslab�n
% Articulaci�n 5
vc55 = v55+cross(w55,s55); % Suponiendo que el centro de gravedad se mueva a la misma velocidad que el eslab�n
% Articulaci�n 6
vc66 = v66+cross(w66,s66); % Suponiendo que el centro de gravedad se mueva a la misma velocidad que el eslab�n


% Energ�a cin�tica
% K = sum_i=1_N Ki
%  donde
%         Ki = 0.5*mi*vcii'*vcii+0.5*wii'*Iii*wii;
K1=0.5*m1*vc11'*vc11+ 0.5* w11'*I11*w11;
K2=0.5*m2*vc22'*vc22+ 0.5* w22'*I22*w22;
K3=0.5*m3*vc33'*vc33+ 0.5* w33'*I33*w33;
K4=0.5*m4*vc44'*vc44+ 0.5* w44'*I44*w44;
K5=0.5*m5*vc55'*vc55+ 0.5* w55'*I55*w55;
K6=0.5*m6*vc66'*vc66+ 0.5* w66'*I66*w66;
K=K1+K2+K3+K4+K5+K6;
% Enecrg�a potencial
% U = sum_i=1_N Ui
%  donde
%         Ui = mi*g*Zi (suponiendo g en direcci�n a eje Z fijo)
T01=[ cos(theta1) -cos(alpha1)*sin(theta1) sin(alpha1)*sin(theta1) a1*cos(theta1);
    sin(theta1)  cos(alpha1)*cos(theta1)  -sin(alpha1)*cos(theta1) a1*sin(theta1);
    0            sin(alpha1)              cos(alpha1)              d1            ;
    0                  0                     0                      1            ];

T12=[ cos(theta2) -cos(alpha2)*sin(theta2) sin(alpha2)*sin(theta2) a2*cos(theta2);
    sin(theta2)  cos(alpha2)*cos(theta2)  -sin(alpha2)*cos(theta2) a2*sin(theta2);
    0            sin(alpha2)              cos(alpha2)              d2            ;
    0                  0                     0                      1            ];

T23=[ cos(theta3) -cos(alpha3)*sin(theta3) sin(alpha3)*sin(theta3) a3*cos(theta3);
    sin(theta3)  cos(alpha3)*cos(theta3)  -sin(alpha3)*cos(theta3) a3*sin(theta3);
    0            sin(alpha3)              cos(alpha3)              d3            ;
    0                  0                     0                      1            ];
T34=[ cos(theta4) -cos(alpha4)*sin(theta4) sin(alpha4)*sin(theta4) a4*cos(theta4);
    sin(theta4)  cos(alpha4)*cos(theta4)  -sin(alpha4)*cos(theta4) a4*sin(theta4);
    0            sin(alpha4)              cos(alpha4)              d4            ;
    0                  0                     0                      1            ];
T45=[ cos(theta5) -cos(alpha5)*sin(theta5) sin(alpha5)*sin(theta5) a5*cos(theta5);
    sin(theta5)  cos(alpha5)*cos(theta5)  -sin(alpha5)*cos(theta5) a5*sin(theta5);
    0            sin(alpha5)              cos(alpha5)              d5            ;
    0                  0                     0                      1            ];
T56=[ cos(theta6) -cos(alpha6)*sin(theta6) sin(alpha6)*sin(theta6) a6*cos(theta6);
    sin(theta6)  cos(alpha6)*cos(theta6)  -sin(alpha6)*cos(theta6) a6*sin(theta6);
    0            sin(alpha6)              cos(alpha6)              d6            ;
    0                  0                     0                      1            ];



Pc01=T01*[eye(3) s11; [0 0 0] 1];
U1=m1*g*Pc01(3,4);
Pc02=T01*T12*[eye(3) s22; [0 0 0] 1];
U2=m2*g*Pc02(3,4);
Pc03=T01*T12*T23*[eye(3) s33; [0 0 0] 1];
U3=m3*g*Pc03(3,4);
Pc04=T01*T12*T23*T34*[eye(3) s44; [0 0 0] 1];
U4=m4*g*Pc04(3,4);
Pc05=T01*T12*T23*T34*T45*[eye(3) s55; [0 0 0] 1];
U5=m5*g*Pc05(3,4);
Pc06=T01*T12*T23*T34*T45*T56*[eye(3) s66; [0 0 0] 1];
U6=m6*g*Pc06(3,4);
U=U1+U2+U3+U4+U5+U6;

L=K-U;

% A partir de la Lagrangiana, calcular las ecuaciones
% Ecuaci�n 1: T1 = d/dt(dL/qd1) - dL/q1
dLdq1 = diff(L,qd1);
dLdq1dt = diff(dLdq1,q1)*qd1+diff(dLdq1,q2)*qd2+diff(dLdq1,q3)*qd3+diff(dLdq1,qd1)*qdd1+diff(dLdq1,qd2)*qdd2+diff(dLdq1,qd3)*qdd3+diff(dLdq1,q4)*qd1+diff(dLdq1,q5)*qd5+diff(dLdq1,q6)*qd6+diff(dLdq1,qd4)*qdd1+diff(dLdq1,qd5)*qdd5+diff(dLdq1,qd6)*qdd6;
dLdq1 = diff(L,q1);
T1 = dLdq1dt - dLdq1;

% Ecuaci�n 2: T2 = d/dt(dL/qd2) - dL/q2
dLdq2 = diff(L,qd2);
dLdq2dt = diff(dLdq2,q1)*qd1+diff(dLdq2,q2)*qd2+diff(dLdq2,q3)*qd3+diff(dLdq2,qd1)*qdd1+diff(dLdq2,qd2)*qdd2+diff(dLdq2,qd3)*qdd3+diff(dLdq2,q4)*qd1+diff(dLdq2,q5)*qd5+diff(dLdq2,q6)*qd6+diff(dLdq2,qd4)*qdd1+diff(dLdq2,qd5)*qdd5+diff(dLdq2,qd6)*qdd6;
dLdq2 = diff(L,q2);
T2 = dLdq2dt - dLdq2;

% Ecuaci�n3: T3 = d/dt(dL/qd3) - dL/q3
dLdq3 = diff(L,qd3);
dLdq3dt = diff(dLdq3,q1)*qd1+diff(dLdq3,q2)*qd2+diff(dLdq3,q3)*qd3+diff(dLdq3,qd1)*qdd1+diff(dLdq3,qd2)*qdd2+diff(dLdq3,qd3)*qdd3+diff(dLdq3,q4)*qd1+diff(dLdq3,q5)*qd5+diff(dLdq3,q6)*qd6+diff(dLdq3,qd4)*qdd1+diff(dLdq3,qd5)*qdd5+diff(dLdq3,qd6)*qdd6;
dLdq3 = diff(L,q3);
T3 = dLdq3dt - dLdq3;

% Ecuaci�n4: T4 = d/dt(dL/qd4) - dL/q4
dLdq4 = diff(L,qd4);
dLdq4dt = diff(dLdq4,q1)*qd1+diff(dLdq4,q2)*qd2+diff(dLdq4,q3)*qd3+diff(dLdq4,qd1)*qdd1+diff(dLdq4,qd2)*qdd2+diff(dLdq4,qd3)*qdd3+diff(dLdq4,q4)*qd1+diff(dLdq4,q5)*qd5+diff(dLdq4,q6)*qd6+diff(dLdq4,qd4)*qdd1+diff(dLdq4,qd5)*qdd5+diff(dLdq4,qd6)*qdd6;
dLdq4 = diff(L,q4);
T4 = dLdq4dt - dLdq4;

% Ecuaci�n4: T5 = d/dt(dL/qd5) - dL/q5
dLdq5 = diff(L,qd5);
dLdq5dt = diff(dLdq5,q1)*qd1+diff(dLdq5,q2)*qd2+diff(dLdq5,q3)*qd3+diff(dLdq5,qd1)*qdd1+diff(dLdq5,qd2)*qdd2+diff(dLdq5,qd3)*qdd3+diff(dLdq5,q4)*qd1+diff(dLdq5,q5)*qd5+diff(dLdq5,q6)*qd6+diff(dLdq5,qd4)*qdd1+diff(dLdq5,qd5)*qdd5+diff(dLdq5,qd6)*qdd6;
dLdq5 = diff(L,q5);
T5 = dLdq5dt - dLdq5;

% Ecuaci�n4: T6 = d/dt(dL/qd6) - dL/q6
dLdq6 = diff(L,qd6);
dLdq6dt = diff(dLdq6,q1)*qd1+diff(dLdq6,q2)*qd2+diff(dLdq6,q3)*qd3+diff(dLdq6,qd1)*qdd1+diff(dLdq6,qd2)*qdd2+diff(dLdq6,qd3)*qdd3+diff(dLdq6,q4)*qd1+diff(dLdq6,q5)*qd5+diff(dLdq6,q6)*qd6+diff(dLdq6,qd4)*qdd1+diff(dLdq6,qd5)*qdd5+diff(dLdq6,qd6)*qdd6;
dLdq6 = diff(L,q6);
T6 = dLdq6dt - dLdq6;


%%% MANIPULACI�N SIMB�LICA DE LAS ECUACIONES %%%
% En ecuaciones matriciales (solo parte del brazo):
%
% T= M(q)qdd+V(q,qd)+G(q) = M(q)qdd+VG(q,qd)
%

% Primera ecuaci�n
% -----------------
% C�lculo de los t�rminos de la matriz de inercia (afines a qdd)
M11 = diff(T1,qdd1);
Taux = (T1 - M11*qdd1);
M12 = diff(Taux,qdd2);
Taux = (Taux-M12*qdd2);
M13= diff(Taux,qdd3);
Taux = (Taux-M13*qdd3);
M14= diff(Taux,qdd4);
Taux = (Taux-M14*qdd4);
M15= diff(Taux,qdd5);
Taux = (Taux-M15*qdd5);
M16= diff(Taux,qdd6);
Taux = (Taux-M16*qdd6);
% Taux restante contiene t�rminos Centr�petos/Coriolis y Gravitatorios
% T�rminos gravitatorios: dependen linealmente de "g"
G1=diff(Taux,g)*g;
Taux=(Taux-G1);
% Taux restante contiene t�rminos Centr�petos/Coriolis
V1=Taux;

% Segunda ecuaci�n
% -----------------
% C�lculo de los t�rminos de la matriz de inercia (afines a qdd)
M21 = diff(T2,qdd1);
Taux = (T2 - M21*qdd1);
M22 = diff(Taux,qdd2);
Taux = (Taux-M22*qdd2);
M23 = diff(Taux,qdd3);
Taux = (Taux-M23*qdd3);
M24 = diff(Taux,qdd4);
Taux = (Taux-M24*qdd4);
M25 = diff(Taux,qdd5);
Taux = (Taux-M25*qdd5);
M26 = diff(Taux,qdd6);
Taux = (Taux-M26*qdd6);
% Taux restante contiene t�rminos Centr�petos/Coriolis y Gravitatorios
% T�rminos gravitatorios: dependen linealmente de "g"
G2=diff(Taux,g)*g;
Taux=(Taux-G2);
% Taux restante contiene t�rminos Centr�petos/Coriolis
V2=Taux;

% Tercera ecuaci�n
% -----------------
% C�lculo de los t�rminos de la matriz de inercia (afines a qdd)
M31 = diff(T3,qdd1);
Taux = (T3 - M31*qdd1);
M32 = diff(Taux,qdd2);
Taux = (Taux-M32*qdd2);
M33 = diff(Taux,qdd3);
Taux = (Taux-M33*qdd3);
M34 = diff(Taux,qdd4);
Taux = (Taux-M34*qdd4);
M35 = diff(Taux,qdd5);
Taux = (Taux-M35*qdd5);
M36 = diff(Taux,qdd6);
Taux = (Taux-M36*qdd6);
% Taux restante contiene t�rminos Centr�petos/Coriolis y Gravitatorios
% T�rminos gravitatorios: dependen linealmente de "g"
G3=diff(Taux,g)*g;
Taux=(Taux-G3);
% Taux restante contiene t�rminos Centr�petos/Coriolis
V3=Taux;

% Cuarta ecuaci�n
% -----------------
% C�lculo de los t�rminos de la matriz de inercia (afines a qdd)
M41 = diff(T4,qdd1);
Taux = (T4 - M41*qdd1);
M42 = diff(Taux,qdd2);
Taux = (Taux-M42*qdd2);
M43 = diff(Taux,qdd3);
Taux = (Taux-M43*qdd3);
M44 = diff(Taux,qdd4);
Taux = (Taux-M44*qdd4);
M45 = diff(Taux,qdd5);
Taux = (Taux-M45*qdd5);
M46 = diff(Taux,qdd6);
Taux = (Taux-M46*qdd6);
% Taux restante contiene t�rminos Centr�petos/Coriolis y Gravitatorios
% T�rminos gravitatorios: dependen linealmente de "g"
G4=diff(Taux,g)*g;
Taux=(Taux-G4);
% Taux restante contiene t�rminos Centr�petos/Coriolis
V4=Taux;

% Quinta ecuaci�n
% -----------------
% C�lculo de los t�rminos de la matriz de inercia (afines a qdd)
M51 = diff(T5,qdd1);
Taux = (T5 - M51*qdd1);
M52 = diff(Taux,qdd2);
Taux = (Taux-M52*qdd2);
M53 = diff(Taux,qdd3);
Taux = (Taux-M53*qdd3);
M54 = diff(Taux,qdd4);
Taux = (Taux-M54*qdd4);
M55 = diff(Taux,qdd5);
Taux = (Taux-M55*qdd5);
M56 = diff(Taux,qdd6);
Taux = (Taux-M56*qdd6);
% Taux restante contiene t�rminos Centr�petos/Coriolis y Gravitatorios
% T�rminos gravitatorios: dependen linealmente de "g"
G5=diff(Taux,g)*g;
Taux=(Taux-G5);
% Taux restante contiene t�rminos Centr�petos/Coriolis
V5=Taux;

% Sexta ecuaci�n
% -----------------
% C�lculo de los t�rminos de la matriz de inercia (afines a qdd)
M61 = diff(T6,qdd1);
Taux = (T6 - M61*qdd1);
M62 = diff(Taux,qdd2);
Taux = (Taux-M62*qdd2);
M63 = diff(Taux,qdd3);
Taux = (Taux-M63*qdd3);
M64 = diff(Taux,qdd4);
Taux = (Taux-M64*qdd4);
M65 = diff(Taux,qdd5);
Taux = (Taux-M65*qdd5);
M66 = diff(Taux,qdd6);
Taux = (Taux-M66*qdd6);
% Taux restante contiene t�rminos Centr�petos/Coriolis y Gravitatorios
% T�rminos gravitatorios: dependen linealmente de "g"
G6=diff(Taux,g)*g;
Taux=(Taux-G6);
% Taux restante contiene t�rminos Centr�petos/Coriolis
V6=Taux;

% Simplificaci�n de expresiones
% M11=(M11); M12=(M12); M13=(M13);M14=(M14); M15=(M15); M16=(M16);
% M21=(M21); M22=(M22); M23=(M23);M24=(M24); M25=(M25); M26=(M26);
% M31=(M31); M32=(M32); M33=(M33);M34=(M34); M35=(M35); M36=(M36);
% M41=(M41); M42=(M42); M43=(M43);M44=(M44); M45=(M45); M46=(M46);
% M51=(M51); M52=(M52); M53=(M53);M54=(M54); M55=(M55); M56=(M56);
% M61=(M61); M62=(M62); M63=(M63);M64=(M64); M65=(M65); M66=(M66);
% 
% 
% V1=(V1); V2=(V2); V3=(V3);V4=(V4); V5=(V5); V6=(V6);
% G1=(G1); G2=(G2); G3=(G3);G4=(G4); G5=(G5); G6=(G6);

% Apilaci�n en matrices y vectores
M = [M11 M12 M13 M14 M15 M16; M21 M22 M23 M24 M25 M26; M31 M32 M33 M34 M35 M36; M41 M42 M43 M44 M45 M46;M51 M52 M53 M54 M55 M56;M61 M62 M63 M64 M65 M66];
V = [V1; V2; V3; V4; V5; V6];
G = [G1; G2; G3; G4; G5; G6];


% Inclusi�n de los motores en la ecuaci�n din�mica
%
% T= Ma(q)qdd+Va(q,qd)+Ga(q)
%
% Ma = M + R^2*Jm     Va=V + R^2*Bm*qd     Ga=G
%
R=diag([R1 R2 R3 R4 R5 R6]);
Jm=diag([Jm1 Jm2 Jm3 Jm4 Jm5 Jm6]);
Bm=diag([Bm1 Bm2 Bm3 Bm4 Bm5 Bm6]);
% Kt=diag([Kt1 Kt2 Kt3]); % No utilizado

Ma=M+R*R*Jm;
Va=V+R*R*Bm*[qd1 ; qd2 ; qd3; qd4; qd5; qd6];
Ga = G;

% La funci�n vpa del Symbolic Toolbox evalua las expresiones de las
% fracciones de una funci�n simb�lica, redonde�ndolas con la precisi�n que podr�a pasarse como segundo
% argumento.
Ma_lag=vpa(Ma,5);
Va_lag=vpa(Va,5);
Ga_lag=vpa(G,5);
