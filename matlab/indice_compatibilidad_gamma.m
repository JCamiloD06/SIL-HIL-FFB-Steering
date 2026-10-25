%% indice_compatibilidad_gamma.m
% Computation of the Spectral Compatibility Index (Gamma_E) and linear regression.
% Author: Juan Camilo Diaz-Lopez
% Department of Mechanical and Mechatronics Engineering
% Universidad Nacional de Colombia, Sede La Paz
% Research Paper: Physical Steering Dynamics and Force-Feedback Loading
%                 in Autonomous Path Tracking (Ingenieria e Investigacion)
%
clear; clc; close all;
raiz = fileparts(fileparts(mfilename('fullpath')));
dir_res = fullfile(raiz, 'results', 'exploratory', 'compatibilidad');
dir_fig = fullfile(raiz, 'figures', 'draft', 'compatibilidad');
if ~exist(dir_res, 'dir'), mkdir(dir_res); end
if ~exist(dir_fig, 'dir'), mkdir(dir_fig); end

% 1. Parámetros identificados del modelo G2(s) (P2DU formal, Fase 1)
fn_act = 2.5354; % Hz
zeta_act = 0.26412;
K_act = 1.0439;
Td_act = 0.0063757; % s (retardo puro)
wn_act = 2 * pi * fn_act;
f_BW_act = 3.7507; % Hz (-3 dB)
slew_lim_dps = 560.0; % deg/s
deadband_deg = 0.5;   % deg (zona muerta de fricción / stiction c01)
kw = 19.4888;         % deg volante por deg rueda (900 deg)

s = tf('s');
G_act = (K_act * wn_act^2) / (s^2 + 2 * zeta_act * wn_act * s + wn_act^2) * exp(-Td_act * s);

% 2. Cargar densidades espectrales de los comandos SIL (s14)
f_psd = fullfile(raiz, 'results', 'exploratory', 'espectro_comando', 's14_psd.csv');
assert(isfile(f_psd), 'Falta s14_psd.csv: corre antes s14');
T_psd = readtable(f_psd);

% Cargar resumen de f95
f_corr = fullfile(raiz, 'results', 'exploratory', 'espectro_comando', 's14_corridas.csv');
T_corr = readtable(f_corr);

% Filtrar corridas únicas por (controlador, perfil)
corridas = unique(T_psd(:, {'controlador', 'perfil'}), 'rows');
nc = height(corridas);

resultados = cell(nc, 10);

fprintf('========================================================================================================================\n');
fprintf('%-18s | %-12s | %9s | %10s | %9s | %10s | %10s | %9s | %8s\n', ...
    'Controlador', 'Perfil', 'Gamma_E', 'Gamma_sim', 'Lambda', 'E_>BW (%)', 'Gamma_rate', 'Sat_Tasa', 'f95 (Hz)');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');

fig_spec = figure('Name', 'Espectro vs Error Actuador', 'Position', [100 100 1100 650], 'Color', 'w');
subplot(2,1,1); hold on; grid on; box on;
subplot(2,1,2); hold on; grid on; box on;

colores = struct('pure_pursuit', [0.15 0.65 0.20], 'stanley', [0.50 0.20 0.75], 'mpc_cinematico', [0.85 0.15 0.15]);
estilos = struct('conservador', '-', 'nominal', '--', 'rapido', ':');

for i = 1:nc
    ctrl = corridas.controlador{i};
    perf = corridas.perfil{i};
    
    sub_psd = T_psd(strcmp(T_psd.controlador, ctrl) & strcmp(T_psd.perfil, perf), :);
    f = sub_psd.f_hz;
    S = sub_psd.S_deg2_hz;
    
    % Frecuencias angulares w = 2*pi*f
    w = 2 * pi * f;
    
    % Respuesta del actuador Ga(jw)
    [mag, phase] = bode(G_act, w);
    mag = squeeze(mag);
    phase = squeeze(phase);
    Ga_jw = mag .* exp(1j * deg2rad(phase));
    
    % Función de transferencia de error de reproducción: E(jw) = 1 - Ga(jw)
    E_jw = 1 - Ga_jw;
    abs_E2 = abs(E_jw).^2;
    
    % Integrales espectrales lineales
    df = [diff(f); f(end) - f(end-1)];
    denom = sum(S .* df);
    num = sum(S .* abs_E2 .* df);
    gamma_E = num / denom;
    
    % Tasa espectral: Sr = w^2 * S
    Sr = w.^2 .* S;
    gamma_rate = sum(Sr .* abs_E2 .* df) / sum(Sr .* df);
    
    % Fracción de energía por encima de f_BW
    idx_bw = f >= f_BW_act;
    E_above_bw = 100 * sum(S(idx_bw) .* df(idx_bw)) / denom;
    
    % f95 desde T_corr
    sub_c = T_corr(strcmp(T_corr.controlador, ctrl) & strcmp(T_corr.perfil, perf) & abs(T_corr.k - 19.5) < 0.5, :);
    if ~isempty(sub_c)
        f95 = sub_c.f95_th_hz(1);
    else
        cum = cumsum(S) / sum(S);
        f95 = f(find(cum >= 0.95, 1));
    end
    lambda = f95 / f_BW_act;
    
    dir_p00 = fullfile(raiz, 'data', 'raw', 'p00_referencia');
    patron = sprintf('campana_s01_%s_%s_*', ctrl, perf);
    d_match = dir(fullfile(dir_p00, patron));
    
    gamma_sim = gamma_E; % Fallback por defecto si no hay corrida temporal
    pct_sat_sim = 0.0;
    
    if ~isempty(d_match)
        f_telem = fullfile(d_match(1).folder, d_match(1).name, 'telemetria.csv');
        if isfile(f_telem)
            try
                T_run = readtable(f_telem);
                idx_med = strcmp(string(T_run.fase_vuelta), "medida");
                if ~any(idx_med), idx_med = true(height(T_run), 1); end
                Tm = T_run(idx_med, :);
                
                t_arr = Tm.t_s;
                th_cmd_arr = rad2deg(Tm.delta_aplicado_rad) * kw;
                
                % Simulación no lineal
                [th_sim_arr, omega_sim_arr] = simular_no_lineal(t_arr, th_cmd_arr, ...
                    K_act, wn_act, zeta_act, Td_act, slew_lim_dps, deadband_deg);
                
                denom_sim = sum((th_cmd_arr - mean(th_cmd_arr)).^2);
                if denom_sim > 1e-6
                    gamma_sim = sum((th_cmd_arr - th_sim_arr).^2) / denom_sim;
                end
                pct_sat_sim = 100 * mean(abs(omega_sim_arr) >= (slew_lim_dps - 1.0));
            catch ME
                fprintf('[AVISO] Error simulando no lineal para %s %s: %s\n', ctrl, perf, ME.message);
            end
        end
    end
    
    fprintf('%-18s | %-12s | %9.4f | %10.4f | %9.4f | %9.2f%% | %10.4f | %8.2f%% | %8.3f Hz\n', ...
        ctrl, perf, gamma_E, gamma_sim, lambda, E_above_bw, gamma_rate, pct_sat_sim, f95);
    
    resultados(i,:) = {ctrl, perf, gamma_E, gamma_sim, lambda, E_above_bw, gamma_rate, pct_sat_sim, f95, f_BW_act};
    
    % Plot espectro en panel superior
    subplot(2,1,1);
    c_col = colores.(ctrl);
    c_est = estilos.(perf);
    plot(f, 10*log10(S + 1e-6), 'Color', c_col, 'LineStyle', c_est, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s (%s)', ctrl, perf));
end
fprintf('========================================================================================================================\n');

% Panel 1: Densidad Espectral de los Controladores
subplot(2,1,1);
xline(fn_act, 'k--', 'LineWidth', 1.8, 'DisplayName', sprintf('Resonancia Actuador f_n=%.2f Hz', fn_act));
xline(f_BW_act, 'b:', 'LineWidth', 1.5, 'DisplayName', sprintf('Ancho de Banda f_{-3dB}=%.2f Hz', f_BW_act));
ylabel('PSD S_\theta(f) [dB deg^2/Hz]', 'FontSize', 10, 'FontWeight', 'bold');
title('Densidad Espectral de Comando SIL vs Resonancia del Actuador', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 10]);
legend('Location', 'northeast', 'FontSize', 8, 'NumColumns', 2);

% Panel 2: Error de reproducción |1 - Ga(jw)|^2
subplot(2,1,2);
f_eval = linspace(0.01, 10, 500);
w_eval = 2 * pi * f_eval;
[m_ev, p_ev] = bode(G_act, w_eval);
G_ev = squeeze(m_ev) .* exp(1j * deg2rad(squeeze(p_ev)));
E2_ev = abs(1 - G_ev).^2;
plot(f_eval, E2_ev, 'k-', 'LineWidth', 2.0, 'DisplayName', '|1 - G_a(j\omega)|^2 (Distorsión)');
xline(fn_act, 'k--', 'LineWidth', 1.5, 'DisplayName', 'f_n resonancia');
ylabel('|1 - G_a(j2\pi f)|^2', 'FontSize', 10, 'FontWeight', 'bold');
xlabel('Frecuencia f [Hz]', 'FontSize', 11, 'FontWeight', 'bold');
title('Función de Ponderación de Distorsión Dinámica del Actuador', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 10]);
ylim([0 6]);
legend('Location', 'northwest', 'FontSize', 9);

exportgraphics(fig_spec, fullfile(dir_fig, 'fig_s29_espectro_vs_actuador.png'), 'Resolution', 600);
exportgraphics(fig_spec, fullfile(dir_fig, 'fig_s29_espectro_vs_actuador.pdf'), 'ContentType', 'vector');

% Exportar CSV consolidado con Gamma_sim
T_out = cell2table(resultados, 'VariableNames', ...
    {'Controlador', 'Perfil', 'Gamma_E', 'Gamma_sim', 'Lambda', 'Pct_Energia_Sobre_BW', ...
     'Gamma_E_rate', 'Pct_Sat_Tasa_Sim', 'f95_cmd_hz', 'f_BW_act_hz'});
writetable(T_out, fullfile(dir_res, 's29_indices_compatibilidad.csv'));

%% Figura 2: Gráfica de Barras Comparativa de Índices (3 Paneles a 600 dpi)
fig_bar = figure('Name', 'Comparativa Indices Compatibilidad', 'Position', [100 100 1200 480], 'Color', 'w');

sub_nom = T_out(strcmp(T_out.Perfil, 'nominal'), :);
ctrl_names = {'Pure Pursuit', 'Stanley', 'MPC Cinemático'};

g_e_nom = [sub_nom.Gamma_E(strcmp(sub_nom.Controlador, 'pure_pursuit')), ...
           sub_nom.Gamma_E(strcmp(sub_nom.Controlador, 'stanley')), ...
           sub_nom.Gamma_E(strcmp(sub_nom.Controlador, 'mpc_cinematico'))];

g_sim_nom = [sub_nom.Gamma_sim(strcmp(sub_nom.Controlador, 'pure_pursuit')), ...
             sub_nom.Gamma_sim(strcmp(sub_nom.Controlador, 'stanley')), ...
             sub_nom.Gamma_sim(strcmp(sub_nom.Controlador, 'mpc_cinematico'))];
       
g_r_nom = [sub_nom.Gamma_E_rate(strcmp(sub_nom.Controlador, 'pure_pursuit')), ...
           sub_nom.Gamma_E_rate(strcmp(sub_nom.Controlador, 'stanley')), ...
           sub_nom.Gamma_E_rate(strcmp(sub_nom.Controlador, 'mpc_cinematico'))];

% Panel 1: Gamma_E (Lineal)
subplot(1,3,1); hold on; grid on; box on;
bar(g_e_nom, 'FaceColor', [0.2 0.45 0.75]);
set(gca, 'XTick', 1:3, 'XTickLabel', ctrl_names, 'FontSize', 10);
ylabel('\Gamma_E (Lineal FRF)', 'FontSize', 11, 'FontWeight', 'bold');
title('\Gamma_E Primario (Lineal)', 'FontSize', 11, 'FontWeight', 'bold');

% Panel 2: Gamma_sim (No lineal: Slew-rate 560 deg/s + Fricción)
subplot(1,3,2); hold on; grid on; box on;
bar(g_sim_nom, 'FaceColor', [0.85 0.45 0.15]);
set(gca, 'XTick', 1:3, 'XTickLabel', ctrl_names, 'FontSize', 10);
ylabel('\Gamma_{sim} (No Lineal con Límites)', 'FontSize', 11, 'FontWeight', 'bold');
title('\Gamma_{sim} (Slew-rate 560°/s + Fricción,', 'FontSize', 11, 'FontWeight', 'bold');

% Panel 3: Gamma_E_rate (Ponderado por tasa)
subplot(1,3,3); hold on; grid on; box on;
bar(g_r_nom, 'FaceColor', [0.80 0.20 0.20]);
set(gca, 'XTick', 1:3, 'XTickLabel', ctrl_names, 'FontSize', 10);
ylabel('\Gamma_{E,rate} (Ponderado por Tasa)', 'FontSize', 11, 'FontWeight', 'bold');
title('\Gamma_{E,rate} (Demanda de Tasa)', 'FontSize', 11, 'FontWeight', 'bold');

sgtitle('Predicción Ex-Ante de Compatibilidad Controlador-Actuador (Plan VSD §9 /', 'FontSize', 13, 'FontWeight', 'bold');

exportgraphics(fig_bar, fullfile(dir_fig, 'fig_s29_indices_gamma.png'), 'Resolution', 600);
exportgraphics(fig_bar, fullfile(dir_fig, 'fig_s29_indices_gamma.pdf'), 'ContentType', 'vector');

fprintf('\nIndices calculados exitosamente.\n');
fprintf('Resultados guardados en: %s\n', fullfile(dir_res, 's29_indices_compatibilidad.csv'));
fprintf('Figuras a 600 dpi guardadas en: %s\n', dir_fig);

%% Función local de simulación no lineal
function [theta_sim, omega_sim] = simular_no_lineal(t, u_cmd, K, wn, zeta, Td, rate_lim, deadband)
    dt = 0.002; % Sub-paso de 2 ms para integración continua de alta fidelidad
    t_sim = (t(1):dt:(t(end) + dt))';
    u_sim = interp1(t, u_cmd, t_sim, 'linear', 'extrap');
    
    % Aplicar retardo puro de transporte
    n_del = round(Td / dt);
    if n_del > 0
        u_del = [repmat(u_sim(1), n_del, 1); u_sim(1:end-n_del)];
    else
        u_del = u_sim;
    end
    
    N = numel(t_sim);
    theta = zeros(N, 1);
    omega = zeros(N, 1);
    theta(1) = u_del(1);
    
    for k = 1:N-1
        e_pos = K * u_del(k) - theta(k);
        
        % Zona muerta por fricción estática (stiction)
        if abs(e_pos) < deadband && abs(omega(k)) < 1.0
            omega(k+1) = 0.0;
        else
            d_omega = (wn^2) * e_pos - 2 * zeta * wn * omega(k);
            w_candidate = omega(k) + d_omega * dt;
            % Límite duro de slew-rate físico
            omega(k+1) = max(-rate_lim, min(rate_lim, w_candidate));
        end
        theta(k+1) = theta(k) + omega(k+1) * dt;
    end
    
    % Muestrear de regreso a los instantes exactos t
    theta_sim = interp1(t_sim, theta, t, 'linear', 'extrap');
    omega_sim = interp1(t_sim, omega, t, 'linear', 'extrap');
end

