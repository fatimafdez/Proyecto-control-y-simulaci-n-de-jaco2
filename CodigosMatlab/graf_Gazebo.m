clearvars; clc; close all
% "aaa" es el nombre con el que se han almacenado dentro de los .mat cada
% uno de los datos correspondientes a cada controlador

% Se cargan los datos
% load('datosSimGazebo_controlPAR.mat');
load('datosSimGazebo_controlPID.mat');
% load('datosSimGazebo_controlEstandar.mat');

% Offset de los datos
inicio = 2;

% Pasar de nanosegundos (valor del tiempo en el .bag obtenido de ros) a
% segundos
nanosec_to_seg = 1e-9;

% Escalar tiempo a segundos
aaa(2:end,1) = aaa(2:end,1)*nanosec_to_seg;

% Obtener el tiempo en que se empieza a tomar datos
time_offset = aaa(inicio,1);

% Hacer que ese sea el 0
aaa(2:end,1) = aaa(2:end,1) - time_offset;

% Coger valores
time = aaa(2:end,1);
q1ref = aaa(2:end,2);
q2ref = aaa(2:end,3);
q3ref = aaa(2:end,4);
q4ref = aaa(2:end,5);
q5ref = aaa(2:end,6);
q6ref = aaa(2:end,7);
q1 = aaa(2:end,8);
q2 = aaa(2:end,9);
q3 = aaa(2:end,10);
q4 = aaa(2:end,11);
q5 = aaa(2:end,12);
q6 = aaa(2:end,13);

qref = [q1ref q2ref q3ref q4ref q5ref q6ref];
q = [q1 q2 q3 q4 q5 q6];

% Graficar
for i = 1:6
    figure(i);plot(time,qref(:,i),time,q(:,i));
    grid;
    legend('Referencia', 'Real');
%     titulo = sprintf("Control par computado - articulacion %d", i);
    titulo = sprintf("Control PID - articulacion %d", i);
%     titulo = sprintf("Control estándar - articulacion %d", i);
    xlabel('Tiempo (s)');
    ylabel('Posición angular (rad)');
    title(titulo);
end


set(gcf,'color','w');