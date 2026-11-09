%% modelo_vehiculo_2dof.m
% Linear 2-DOF lateral bicycle dynamics and understeer gradient Kus.
%
% Author: Juan Camilo Diaz-Lopez
% Universidad Nacional de Colombia, Sede La Paz
% Research Paper: Software-to-Hardware-in-the-Loop Transferability of
%                 Autonomous Path Tracking under Physical Steering Dynamics
%                 and Force-Feedback Loading (Ingenieria e Investigacion)
%
% This script computes the continuous-time state-space lateral dynamics
% and understeer characteristics of the experimental test vehicle
% (Alfa Romeo Giulietta QV) across longitudinal speeds.

clearvars; close all; clc;

%% 1. Vehicle Parameters (Table 1 of Manuscript)
m   = 1320;         % Total vehicle mass [kg]
Iz  = 2200;         % Yaw moment of inertia [kg*m^2]
L   = 2.634;        % Wheelbase [m]
wf  = 1.554;        % Front track width [m]
dist_f = 0.61;      % Static front weight distribution (61/39)

a   = L * (1 - dist_f);  % Distance from CG to front axle [m] (1.027 m)
b   = L * dist_f;        % Distance from CG to rear axle [m] (1.607 m)

% Identified cornering stiffnesses [N/rad]
Cf  = 90000;        % Front axle cornering stiffness
Cr  = 85000;        % Rear axle cornering stiffness

% Static Understeer Gradient [rad/(m/s^2)] and [deg/g]
g = 9.81;
K_us_rad = (m / L) * (b / (2 * Cf) - a / (2 * Cr));
K_us_deg_g = K_us_rad * (180 / pi) * g;

fprintf('=================================================================\n');
fprintf('Vehicle Lateral Dynamics (2-DOF Bicycle Model)\n');
fprintf('=================================================================\n');
fprintf('Mass (m):                     %d kg\n', m);
fprintf('Wheelbase (L):                %.3f m (a = %.3f m, b = %.3f m)\n', L, a, b);
fprintf('Front Cornering Stiffness Cf: %.0f N/rad\n', Cf);
fprintf('Rear Cornering Stiffness Cr:  %.0f N/rad\n', Cr);
fprintf('Understeer Gradient Kus:      %.3f deg/g (Understeer)\n', K_us_deg_g);
fprintf('=================================================================\n\n');

%% 2. State-Space Representation across Speeds
% State vector: x = [lateral velocity vy (m/s); yaw rate r (rad/s)]
% Input: u = front road-wheel angle delta (rad)

speeds_kmh = [40, 60, 80, 100, 120];
speeds_ms  = speeds_kmh / 3.6;

figure('Name', 'Vehicle Lateral Step Response', 'Color', 'w', 'Position', [150 150 900 450]);
t_sim = 0:0.005:2.0;
delta_step = deg2rad(2.0); % 2-degree road-wheel step

subplot(1, 2, 1); hold on; grid on; box on;
subplot(1, 2, 2); hold on; grid on; box on;

for i = 1:length(speeds_ms)
    U = speeds_ms(i);
    
    A = [ -(Cf + Cr)/(m*U),       (-a*Cf + b*Cr)/(m*U) - U; ...
          (-a*Cf + b*Cr)/(Iz*U),  -(a^2*Cf + b^2*Cr)/(Iz*U) ];
    B = [ Cf/m; a*Cf/Iz ];
    C = [ 1, 0; 0, 1 ];
    D = [ 0; 0 ];
    
    sys_2dof = ss(A, B, C, D);
    y_step = step(sys_2dof * delta_step, t_sim);
    
    vy_resp = y_step(:, 1);
    r_resp  = rad2deg(y_step(:, 2));
    
    subplot(1, 2, 1);
    plot(t_sim, vy_resp, 'LineWidth', 1.5, 'DisplayName', sprintf('%d km/h', speeds_kmh(i)));
    
    subplot(1, 2, 2);
    plot(t_sim, r_resp, 'LineWidth', 1.5, 'DisplayName', sprintf('%d km/h', speeds_kmh(i)));
end

subplot(1, 2, 1);
xlabel('Time [s]'); ylabel('Lateral Velocity v_y [m/s]');
title('Lateral Velocity Transient Response');
legend('Location', 'southeast');

subplot(1, 2, 2);
xlabel('Time [s]'); ylabel('Yaw Rate r [deg/s]');
title('Yaw Rate Transient Response');
legend('Location', 'southeast');
