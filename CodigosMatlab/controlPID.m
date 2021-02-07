% Archivo para calculos de diseño frecuencial de controladores. CALCULO DE
% PD MEDIANTE TECNICAS FRECUENCIALES (BODE). ES NECESARIO TENER LAS
% FUNCIONES DE TRANSFERENCIA DE CADA ESLABÓN, ASÍ COMO LAS ESPECIFICACIONES
% DEL CONTROL (wc, Mfdes). CAMBIAR wc A LA QUE SE NECESITE. LAS VARIABLES
% MFDESEADO, MFACTUAL, Y GANANCIA ACTUAL SE PIDEN POR TECLADO EN LA VENTANA
% DE COMANDOS DE MATLAB

clear all;clc;

PI = 0;                 % Variable para indicar si se necesita un PI (=1) o un PID (=0)
ganan=[];               % Se almacenarán las ganancias

load('JACO_2_funcionesTransferencia.mat'); % Hay que cargar la funcion de transferencia

for i=1:3
    
    if(i==1)
        G=G1;
    elseif(i==2)
        G=G2;
    elseif(i==3)
        G=G3;
    elseif(i==4)
        G=G4;
    elseif(i==5)
        G=G5;
    else
        G=G6;
    end
    
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
    angdes = Mfdes - Mfact + 5;
    angdesrad = angdes * pi/180;
    
    if (angdes<=-35)
        mensaje=sprintf("Se necesita un PI")
        PI = 1;
    else
        mensaje=sprintf("Se necesita un PID")
        PI = 0;
    end
    
    if (angdes>90)
        mensaje=sprintf("El angulo deseado es superior a 90, disminuir Mfdes")
    end
    
    % Calculo de tau (en radianes)
    if (PI == 0)
        tau = tan((angdesrad + pi/2)/2) / wc
    else
        tau = tan(angdesrad + pi/2)/ wc
    end
    
    % Diseño del controlador en funcion de la necesidad de PI o PID
    if (PI == 1)
        C = tf([tau 1],[tau 0]);
    else
        C = tf(conv([tau 1],[tau 1]),[tau 0]);
    end
    
    bode(C*G,logspace(0,3,100));grid;   % Se dibuja de nuevo Gba, ahora con el controlador calculado arriba
    
    % Una vez calculado tau, se pasa a ajustar la ganancia.
    prompt2 = 'Introducir la ganancia para wc: ';
    g = input(prompt2); % Se lee la ganancia
    kp = 10 ^ (-g/20)   % Ajuste de ganancia
    
    % Una vez se tiene tau y kp, se crea el controlador en funcion de la
    % necesidad de tener un PI o un PID
    taf = 1/(10*wc);
    if (PI == 1)
        C = tf(kp*[tau 1],conv([taf 1],[tau 0]))
    else
        C = tf(kp*conv([tau 1],[tau 1]),conv([tau 0],[taf 1]))
    end
    
    
    
    bode(C*G,logspace(0,3,100));grid;   % Se dibuja Gba, comprobando que se cumple el diseño de Mfdes y wc
    ganan=[ganan;kp];   % Almacenamiento de la k
end
kG1 = ganan(1);
kG2 = ganan(2);
kG3 = ganan(3);
kG4 = ganan(4);
kG5 = ganan(5);
kG6 = ganan(6);
% save('kG.mat','kG1','kG2','kG3','kG4','kG5','kG6');    % Guardamos el fichero mat para tenerlo





