# Laboratorio 05 Robótica - Pincher Phantom X100 - ROS Humble - RVIZ

## Presentado por Juan Esteban Otavo García y Ian Saonni Rodríguez Pulido

# Cinemàtica Directa 

Para la construcción de la cinemática directa del robot, se aprovecharon los modelos 3D y los planos del robot Phantom Pincher proveídos por el monitor para la obtención de las longitudes de los eslabones.

A continuaciòn se presenta el código en MATLAB que se usó para determinar la cinemática directa del robot Pincher y definir la tabla de parametros de Denavit Hatenberg.
```

% Definir los parámetros DH

L1 = Link('d', 0.072,  'a', 0,   'alpha', -pi/2, 'offset', 0);
L2 = Link('d', 0,   'a', 0.125, 'alpha', 0,     'offset', -pi/2);
L3 = Link('d', 0,   'a', 0.125, 'alpha', 0,     'offset', 0);
L4 = Link('d', 0,   'a', 0.125,   'alpha', 0, 'offset', 0);
L5 = Link('d', 0,   'a', 0.125,   'alpha', 0, 'offset', 0);

% Crear el modelo del robot
PhantomX = SerialLink([L1 L2 L3 L4 L5], 'name', 'PhantomX');

% Mostrar el robot en la posición inicial
q = [0, 0, 0, 0, 0];
PhantomX.plot(q);
T = PhantomX.fkine(q);
disp('Nueva posición del efector final:');

```
La tabla de parámetros DH obtenida es la siguiente:

| j  | theta | d     | a     | alpha  | offset |
|----|-------|-------|-------|--------|--------|
| 1  | q1    | 0.072 | 0     | -1.571 | 0      |
| 2  | q2    | 0     | 0.125 | 0      | -1.571 |
| 3  | q3    | 0     | 0.125 | 0      | 0      |
| 4  | q4    | 0     | 0.125 | 0      | 0      |
| 5  | q5    | 0     | 0.125 | 0      | 0      |

Utilizando la función PhantomX.plot(q) se puede observar un gráfico del robot acorde a los parámetros DH definidos y a los ángulos escogidos para cada articulación en el vector q. El gráfico del robot en la posición de home se observa a continuación:

![image](https://github.com/jotavo21/Lab05_Robotica_2025_2/blob/main/imagenes/pincher.png)
Finalmente, aprovechando la tabla de parametros, se utiliza PhantomX.fikne(q) para obtener la cinemática directa, es decir, la matriz MTH correspondiente a la posición y orientación del efector final, la cual se muestra a continuación:

![image](https://github.com/jotavo21/Lab05_Robotica_2025_2/blob/main/imagenes/MTH.png)


# Cinemàtica inversa
A continuaciòn se presenta el código en MATLAB que se usó para determinar la cinemática inversa del robot Pincher

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
