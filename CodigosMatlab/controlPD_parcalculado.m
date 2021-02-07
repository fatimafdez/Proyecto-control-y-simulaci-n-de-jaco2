% Archivo para calculos de diseño frecuencial de controladores. CALCULO DE
% PD MEDIANTE TECNICAS FRECUENCIALES (BODE). 

% ES NECESARIO TENER LAS FUNCIONES DE TRANSFERENCIA DE CADA ESLABÓN, ASÍ 
% COMO LAS ESPECIFICACIONES DEL CONTROL (wc, Mfdes). 

% CAMBIAR wc A LA QUE SE NECESITE. LAS VARIABLES MFDESEADO, MFACTUAL, Y 
% GANANCIA ACTUAL SE PIDEN POR TECLADO EN LA VENTANA DE COMANDOS DE MATLAB. 

% CAMBIAR TAMBIEN EL ALCANCE DEL BODE DENTRO DE LOGSPACE(X,Y,N PUNTOS), CON
% X TAL QUE 10^X, Y TAL QUE 10^Y

clear all;close all;

G=tf(1,[1 0 0]) % Funcion de transferencia para calcular el 

numC=1;
denC=1;

C=tf(numC,denC);    % Se genera ft

bode(C*G,logspace(0,3,100));grid;   % Bode Gba con C=1

prompt11 = 'Introduzca el margen de fase deseado: ';
Mfdes = input(prompt11);    % Se lee la ventana de comandos

wc = 104;                   % Frecuencia de corte deseada, constante en los 3 controladores

% Introducir a continuación el valor de Mf actual
prompt1 = 'Introduzca la fase actual: ';
fase = input(prompt1);      % Se lee la fase actual, vista en el bode anteriormente dibujado

Mfact = 180 + fase;
angdes = Mfdes - Mfact;
angdesrad = angdes * pi/180;

% Calculo de tau (en radianes)
tau = tan(angdesrad)/ wc

% Diseño del controlador PD
C = tf([tau 1],1)

bode(C*G,logspace(0,3,100));grid;   % Se dibuja de nuevo Gba, ahora con el controlador calculado arriba

% Una vez calculado tau, se pasa a ajustar la ganancia.
prompt2 = 'Introducir la ganancia para wc: ';
g = input(prompt2); % Se lee la ganancia
kp = 10 ^ (-g/20)   % Ajuste de ganancia

% Una vez se tiene tau y kp, se crea el controlador en funcion de la
% necesidad de tener un PI o un PID
taf = 1/(10*wc);
C = tf(kp*[tau 1],1)

bode(C*G,logspace(0,3,100));grid;   % Se dibuja Gba, comprobando que se cumple el diseño de Mfdes y wc








