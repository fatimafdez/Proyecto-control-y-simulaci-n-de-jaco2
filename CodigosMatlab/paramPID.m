% Archivo para calcular los parámetros del PID a partir de los
% controladores ya calculados. ES NECESARIO TENER LOS CONTROLADORES PID
% CALCULADOS DE ANTEMANO PARA UTILIZAR ESTE SCRIPT. EN LUGAR DE CARGAR EL
% ARCHIVO MAT, PUEDEN SER ESCRITOS A MANO


load('kGanancias.mat');

Ti1 = zeros(6,1);
Td1 = zeros(6,1);
Kp1 = zeros(6,1);
% Pasamos las funciones de transferencia a la forma G(s)=K/[(tau·s+1)·s]
for i = 1:6
    % C1
    [nC1,dC1]=tfdata(C(i));   % Obtenemos informacion de C1
    nC1=nC1{1};             % Cambiamos a un vector, para poder operar
    dC1=dC1{1};
    
    dC1=dC1*(1/nC1(3));      % Buscamos obtener la forma arriba escrita
    nC1=nC1*(1/nC1(3));
    
    Cnuevo1=tf(nC1,dC1);     % Comprobamos que tenemos la forma
    tau1=abs(roots(nC1));    % Calculamos ambas tau
    
    Ti1(i)=nC1(2);                        % Calculo de variables del PID
    Td1(i)=nC1(1)/Ti1(i);
    Kp1(i)=1/dC1(2)*(Ti1(i));
end

Ti=mean(Ti1);
Td=mean(Td1);
% clearvars -except Ti Td Kp1 Kp2 Kp3
