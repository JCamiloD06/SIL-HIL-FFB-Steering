%% identificacion_actuador.m
% Parametric steering actuator model and frequency response characterization.
%
% Author: Juan Camilo Diaz-Lopez
% Universidad Nacional de Colombia, Sede La Paz
% Research Paper: Software-to-Hardware-in-the-Loop Transferability of
%                 Autonomous Path Tracking under Physical Steering Dynamics
%                 and Force-Feedback Loading (Ingenieria e Investigacion)
%
% This script evaluates the continuous-time identified transfer function Ga(s)
% of the direct-drive rotary steering actuator, comparing nominal vs delay-loaded
% dynamics and computing bandwidth, natural frequency, and phase margin.

clearvars; close all; clc;

%% 1. Identified Parameters (Section 2.3 of Manuscript)
K   = 1.044;        % Static gain
wn  = 15.93;        % Natural frequency [rad/s] (fn = 2.535 Hz)
zeta = 0.264;       % Damping ratio
Td  = 0.00638;      % Pure transport and communication delay [s] (6.38 ms)
fbw = 2.78;         % Measured -3 dB bandwidth [Hz]

% Hardware limits
rate_limit_hw = 560.0; % Handwheel slew-rate limit [deg/s]
kw = 19.49;            % Steering column transmission ratio [deg_hw / deg_rw]
rate_limit_rw = rate_limit_hw / kw; % Front road-wheel limit: 28.73 deg/s

fprintf('=================================================================\n');
fprintf('Steering Actuator Identified Transfer Function Ga(s)\n');
fprintf('=================================================================\n');
fprintf('Static Gain (K):              %.3f\n', K);
fprintf('Natural Frequency (wn):       %.2f rad/s (%.2f Hz)\n', wn, wn/(2*pi));
fprintf('Damping Ratio (zeta):         %.3f\n', zeta);
fprintf('Pure Transport Delay (Td):    %.2f ms\n', Td * 1000);
fprintf('Bandwidth (-3 dB):            %.2f Hz\n', fbw);
fprintf('Handwheel Slew-Rate Limit:    %.1f deg/s\n', rate_limit_hw);
fprintf('Road-Wheel Slew-Rate Limit:   %.2f deg/s\n', rate_limit_rw);
fprintf('=================================================================\n\n');

%% 2. Transfer Function Formulation
s = tf('s');
Ga_nominal = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2);
Ga_delayed = Ga_nominal * exp(-s * Td);

%% 3. Frequency Response (Bode Plot)
figure('Name', 'Steering Actuator Frequency Response', 'Color', 'w', 'Position', [150 150 850 500]);
w_vec = 2 * pi * logspace(-1, 2, 500); % 0.1 Hz to 100 Hz

[mag_nom, phase_nom] = bode(Ga_nominal, w_vec);
[mag_del, phase_del] = bode(Ga_delayed, w_vec);

mag_nom = squeeze(mag_nom); phase_nom = squeeze(phase_nom);
mag_del = squeeze(mag_del); phase_del = squeeze(phase_del);
freq_hz = w_vec / (2 * pi);

subplot(2, 1, 1);
semilogx(freq_hz, 20*log10(mag_del), 'b-', 'LineWidth', 2, 'DisplayName', 'Ga(s) Identified');
hold on; grid on; box on;
yline(-3, 'r--', 'LineWidth', 1.2, 'DisplayName', '-3 dB Bandwidth threshold');
xline(fbw, 'k:', 'LineWidth', 1.2, 'DisplayName', sprintf('f_{bw} = %.2f Hz', fbw));
xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
title('Actuator Frequency Response (Bode Diagram)');
legend('Location', 'southwest');

subplot(2, 1, 2);
semilogx(freq_hz, phase_del, 'b-', 'LineWidth', 2, 'DisplayName', 'Ga(s) with Delay (Td = 6.38 ms)');
hold on; grid on; box on;
semilogx(freq_hz, phase_nom, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Ga(s) without Delay');
xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
title('Phase Angle and Delay-Induced Phase Roll-Off');
legend('Location', 'southwest');
