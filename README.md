# Laboratorio 05 Robótica - Pincher Phantom X100 - ROS Humble - RVIZ

## Presentado por Juan Esteban Otavo García y Ian Saonni Rodríguez Pulido

#Cinemàtica Directa 
```
function [theta_0, theta_1, theta_2, theta_3, theta_4] = inverse_kinematics(X, Y, Z, phi)
    
    theta_4 = phi;
    
    theta_0=atan2(Y,X);
    
    Xa=sqrt(X^2+Y^2);
    
    % Longitudes de los eslabones
    link0 = 0.072;
    link1 = 0.125;
    link2 = 0.125;
    link3 = 0.125;

    % Convertir phi a radianes
    phi = deg2rad(phi);
    
    Za=Z-link0;

    % Cálculo de nx y ny
    nx = Xa - link3 * cos(phi);
    nz = Za - link3 * sin(phi);

    % Cálculo de theta2
    delta = nx^2 + nz^2;
    c2 = (delta - link1^2 - link2^2) / (2 * link1 * link2);

    s2 = sqrt(1 - c2^2);

    if imag(s2) ~= 0
        error('El punto dado no se puede alcanzar, intenta con otro valor.');
    end

    theta_2 = atan2(s2, c2);

    % Cálculo de theta1
    s1 = ((link1 + link2 * c2) * nz - link2 * s2 * nx) / delta;
    c1 = ((link1 + link2 * c2) * nx + link2 * s2 * nz) / delta;
    theta_1 = atan2(s1, c1);

    % Cálculo de theta3
    theta_3 = phi - theta_1 - theta_2;

    % Convertir los ángulos a grados
    theta_0 = rad2deg(theta_0);
    theta_1 = rad2deg(theta_1);
    theta_2 = rad2deg(theta_2);
    theta_3 = rad2deg(theta_3);
    theta_4 = theta_4;
end

clc
clear

% Definir las coordenadas y el ángulo phi
X = 0.2;
Y = 0.2;
Z = 0.2;
phi = 45;

% Llamar a la función
[theta_0, theta_1, theta_2, theta_3, theta_4] = inverse_kinematics(X, Y, Z, phi);

% Mostrar los resultados
fprintf('Theta 0: %.2f grados\n', theta_0);
fprintf('Theta 1: %.2f grados\n', theta_1);
fprintf('Theta 2: %.2f grados\n', theta_2);
fprintf('Theta 3: %.2f grados\n', theta_3);
fprintf('Theta 4: %.2f grados\n', theta_4);



% Definir los parámetros DH usando Robotics Toolbox

L1 = Link('d', 0.072,  'a', 0,   'alpha', -pi/2, 'offset', 0);
L2 = Link('d', 0,   'a', 0.125, 'alpha', 0,     'offset', -pi/2);
L3 = Link('d', 0,   'a', 0.125, 'alpha', 0,     'offset', 0);
L4 = Link('d', 0,   'a', 0.125,   'alpha', 0, 'offset', 0);
L5 = Link('d', 0,   'a', 0.125,   'alpha', 0, 'offset', 0);

% Crear el modelo del robot Phantom X
PhantomX = SerialLink([L1 L2 L3 L4 L5], 'name', 'PhantomX');

% Mostrar el robot en la posición inicial
q = [theta_0, theta_1, theta_2, theta_3, theta_4]; % Ángulos en radianes
PhantomX.plot(q);
T = PhantomX.fkine(q);
disp('Nueva posición del efector final:');
disp(T.t);

```
#Cinemàtica inversa
```
clc
clear

% Definir las coordenadas y el ángulo phi
X = 0.2;
Y = 0.2;
Z = 0.2;
phi = 45;

% Llamar a la función
[ theta_1, theta_2, theta_3, theta_4] = ikine(X, Y, Z, phi);

% Mostrar los resultados
fprintf('Theta 1: %.2f grados\n', theta_1);
fprintf('Theta 2: %.2f grados\n', theta_2);
fprintf('Theta 3: %.2f grados\n', theta_3);
fprintf('Theta 4: %.2f grados\n', theta_4);



% Definir los parámetros DH usando Robotics Toolbox

L1 = Link('d', 0,   'a', 0.125, 'alpha', 0,     'offset', -pi/2);
L2 = Link('d', 0,   'a', 0.125, 'alpha', 0,     'offset', 0);
L3 = Link('d', 0,   'a', 0.125,   'alpha', 0, 'offset', 0);
L4 = Link('d', 0,   'a', 0.125,   'alpha', 0, 'offset', 0);

% Crear el modelo del robot Phantom X
PhantomX = SerialLink([L1 L1 L2 L3 L4], 'name', 'PhantomX');

% Mostrar el robot en la posición inicial
q = [0 ,theta_1, theta_2, theta_3, theta_4]; % Ángulos en radianes
PhantomX.plot(q);
T = PhantomX.fkine(q);
disp('Nueva posición del efector final:');
disp(T);

function [ theta_1, theta_2, theta_3, theta_4] = ikine(X, Y, Z, phi)
     
    % cálculo de theta 1
    theta_1 = atan2(Y,X);
    
    r = sqrt(X^2+Y^2);
    
    % Longitudes de los eslabones (m)
    link0 = 0.072;
    link1 = 0.125;
    link2 = 0.125;
    link3 = 0.125;

    % Convertir phi a radianes
    phi = deg2rad(phi);
    
    Za = Z-link0; % Z inicial dedl phantom XPincher

    % Cálculo de r2 y z2
    r2 = r - link3 * cos(phi);
    z2 = Za - link3 * sin(phi);

    % Cálculo de theta3
    l2 = r2^2 + z2^2;
    c2 = (l2 - link1^2 - link2^2) / (2 * link1 * link2);

    s2 = sqrt(1 - c2^2);

    if imag(s2) ~= 0
        error('El punto dado no se puede alcanzar, intenta con otro valor.');
    end

    alpha = atan2(s2, c2);

    theta_3 = pi - alpha;

    % Cálculo de theta 2
    
    beta = atan2(link2*sin(theta_3),link1 + link2*cos(theta_3));
    gamma = atan2(z2,r2);

    theta_2 = gamma + beta;

    % Cálculo de theta4
    theta_4 = phi - theta_2 - theta_3;

    % Convertir los ángulos a grados
    theta_1 = rad2deg(theta_1);
    theta_2 = rad2deg(theta_2);
    theta_3 = rad2deg(theta_3);
    theta_4 = rad2deg(theta_4);
end

```
## Cuadro comparativo entre manipulador Motoman MH6 y manipulador IRB140.
