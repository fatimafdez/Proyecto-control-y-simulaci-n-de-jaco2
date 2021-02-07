%% Parametros obtenidos del fichero xacro dentro del paquete de ROS:
% https://github.com/Kinovarobotics/kinova-ros/blob/master/kinova_description/urdf/kinova_inertial.xacro
clc;clearvars
% Base 0 
cdm_link0 = [0 0 0.1255];
masa_link0 = 0.46784;
alt_link0 = 0.14;
radio_link0 = 0.04;

% Shoulder 1 
cdm_link1 = [0 -0.002 -0.0605];
masa_link1 = 0.7477;
alt_link1 = 0.14;
radio_link1 = 0.04;

% Arm 2 
cdm_link2 = [0 -0.2065 -0.01];
masa_link2 = 0.99;
alt_link2 = 0.4100;
radio_link2 = 0.04;

% Forearm 3
cdm_link3 = [0 0.081 -0.0086];
masa_link3 = 0.6763;
alt_link3 = 0.2073;
radio_link3 = 0.03;

% Wrist 4
cdm_link4 = [0 -0.037 -0.0642];
masa_link4 = 0.426367;
alt_link4 = 0.15;
radio_link4 = 0.04;

% 3-finger hand - 5
cdm_link5 = [0 -0.037 -0.0642];
masa_link5 = 0.99;
alt_link5 = 0.16;
radio_link5 = 0.04;

% Matrices con todos los parámetros
cdm = [cdm_link0; cdm_link1; cdm_link2; cdm_link3; cdm_link4; cdm_link5];
masa = [masa_link0; masa_link1; masa_link2; masa_link3; masa_link4; masa_link5];
alt = [alt_link0; alt_link1; alt_link2; alt_link3; alt_link4; alt_link5];
radios = [radio_link0; radio_link1; radio_link2; radio_link3; radio_link4; radio_link5];
ejes = [0 0 1 1 0 0];

%% Operaciones, nuevamente sacadas del fichero xacro
Ixx = zeros(1, 6);
Iyy = zeros(1, 6);
Izz = zeros(1, 6);

% Depende del valor incluido en el .xacro para los ejes
for i = 1:6
    if ejes(i) == 0
        Ixx(i) = 0.083333 * masa(i) * (3*radios(i)^2 + alt(i)^2);
        Iyy(i) = 0.083333 * masa(i) * (3*radios(i)^2 + alt(i)^2);
        Izz(i) = 0.5      * masa(i) * radios(i)^2;
    else
        Ixx(i) = 0.083333 * masa(i) * (3*radios(i)^2 + alt(i)^2);
        Iyy(i) = 0.5      * masa(i) * radios(i)^2;
        Izz(i) = 0.083333 * masa(i) * (3*radios(i)^2 + alt(i)^2);
    end
end

% Juntar valores de inercia
I_total = [Ixx; Iyy; Izz];

% Matrices de inercia
matrices_inercia = zeros(18, 3);
for i = 1:6
    for j = 1:3
        % Se almacenan cada matriz de inercia una debajo de otra
        matrices_inercia(3 * (i-1) + j, j) = I_total(j, i);
    end
end
clearvars -except cdm radios alt masa I_total matrices_inercia

