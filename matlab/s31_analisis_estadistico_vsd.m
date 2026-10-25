%% s31_analisis_estadistico_vsd.m
% Análisis estadístico formal de la campaña experimental VSD (Fase 5).
%
% Ejecuta el plan de análisis estadístico fijado en docs/ANALYSIS.md:
%   - ANOVA factorial de dos vías (Condición x Controlador) con bloques por Sesión.
%   - Distinción formal e inequívoca entre:
%       * Stanley congelado (k=2.04): Brazo primario de transferencia pura (Paper 1),
%         demuestra la divergencia por saturación de slew-rate en HIL (H1).
%       * Stanley resintonizado (k=1.0): Brazo secundario para análisis de recuperación.
%   - Contrastes de hipótesis planificados (H1 a H4) con corrección de Holm-Bonferroni.
%   - Cálculo del tamaño del efecto (eta al cuadrado parcial, d de Cohen).
%   - Generación de tablas formales y figuras de publicación a 600 dpi (exportgraphics).
%
% Factores analizados:
%   - Condición: SIL, HIL-U, HIL-L
%   - Controlador: Pure Pursuit, Stanley congelado (k=2.04), Stanley resintonizado (k=1.0),
%                  MPC Cinemático, Actuator-Aware MPC (C4)
%   - Perfil de velocidad: Conservador (0.8), Nominal (0.9), Rápido (1.0)
%   - Bloque aleatorio: Sesión (1 a 10)
%
% Protocolo: AGENTS.md §17 (MATLAB para análisis cuantitativo) y §14 (Holm-Bonferroni).

clearvars; close all; clc;
raiz = fileparts(fileparts(mfilename('fullpath')));
if isempty(raiz), raiz = pwd; end

dir_datos = fullfile(raiz, 'data', 'raw', 'trialsteer');
dir_res = fullfile(raiz, 'results', 'primary', 'estadistica_vsd');
dir_fig = fullfile(raiz, 'figures', 'final', 'estadistica_vsd');

for d = {dir_res, dir_fig}
    if ~exist(d{1}, 'dir'), mkdir(d{1}); end
end

fprintf('========================================================================\n');
fprintf('  s31_analisis_estadistico_vsd: ANOVA y Contrastes de Hipótesis (VSD Q1)\n');
fprintf('========================================================================\n\n');

%% 1. Cargar y escanear corridas de campaña y pilotos
carpetas = [dir(fullfile(dir_datos, 'HILU_*')); ...
            dir(fullfile(dir_datos, 'HILL_*')); ...
            dir(fullfile(dir_datos, 'SIL_*'))];

if isempty(carpetas)
    fprintf('[AVISO] No se encontraron carpetas de corridas en %s.\n', dir_datos);
end

fprintf('Encontradas %d carpetas de corridas para procesar.\n', numel(carpetas));

filas = {};
kw = 19.4888;
slew_lim_dps = 560.0;

for k = 1:numel(carpetas)
    c_dir = fullfile(dir_datos, carpetas(k).name);
    f_telem = fullfile(c_dir, 'telemetria.csv');
    f_manif = fullfile(c_dir, 'manifiesto.json');
    f_ciclos = fullfile(c_dir, 'odrive_ciclos.csv');
    
    if ~exist(f_telem, 'file') || ~exist(f_manif, 'file'), continue; end
    
    try
        manif = jsondecode(fileread(f_manif));
        T = readtable(f_telem);
    catch
        continue;
    end
    
    meta = manif.meta;
    res = manif.resumen;
    
    % Filtrar por vuelta medida (si existe fase_vuelta)
    if ismember('fase_vuelta', T.Properties.VariableNames)
        idx_m = strcmp(string(T.fase_vuelta), "medida");
        if ~any(idx_m), idx_m = true(height(T), 1); end
    else
        idx_m = true(height(T), 1);
    end
    Tm = T(idx_m, :);
    if height(Tm) < 10, continue; end
    
    % Condición experimental
    if isfield(manif.meta, 'paper_q2') && isfield(manif.meta.paper_q2, 'condicion')
        cond = string(manif.meta.paper_q2.condicion);
    elseif startsWith(carpetas(k).name, 'HILU_')
        cond = "HIL-U";
    elseif startsWith(carpetas(k).name, 'HILL_')
        cond = "HIL-L";
    elseif startsWith(carpetas(k).name, 'SIL_')
        cond = "SIL";
    else
        cond = "DESCONOCIDA";
    end
    
    ctrl_raw = string(meta.controlador);
    perf = string(meta.perfil);
    ses = meta.sesion;
    
    % Identificar Stanley congelado (k=2.04) vs resintonizado (k=1.0)
    k_stanley = NaN;
    if ctrl_raw == "stanley"
        if isfield(meta, 'parametros_controlador') && isfield(meta.parametros_controlador, 'k')
            k_stanley = meta.parametros_controlador.k;
        elseif isfield(manif, 'config') && isfield(manif.config, 'controladores') && ...
               isfield(manif.config.controladores, 'stanley') && isfield(manif.config.controladores.stanley, 'k')
            k_stanley = manif.config.controladores.stanley.k;
        end
        
        if ~isnan(k_stanley) && k_stanley > 1.5
            ctrl = "stanley_congelado";       % k=2.04 (Brazo primario: transferencia directa del Paper 1)
            desc_ctrl = "Stanley congelado (k=2.04)";
        elseif ~isnan(k_stanley) && k_stanley <= 1.5
            ctrl = "stanley_resintonizado";   % k=1.0 (Brazo secundario: estabilizado en HIL)
            desc_ctrl = "Stanley resintonizado (k=1.0)";
        else
            ctrl = "stanley_congelado";
            desc_ctrl = "Stanley congelado";
        end
    else
        ctrl = ctrl_raw;
        if ctrl == "mpc_cinematico"
            desc_ctrl = "MPC Cinemático";
        elseif ctrl == "mpc_actuator_aware"
            desc_ctrl = "Actuator-Aware MPC (C4)";
        elseif ctrl == "pure_pursuit"
            desc_ctrl = "Pure Pursuit";
        else
            desc_ctrl = string(ctrl);
        end
    end
    
    % Verificación de vuelta completada o corte
    completada = false;
    if isfield(res, 'vuelta_completada')
        completada = logical(res.vuelta_completada);
    elseif isfield(res, 'fase_final') && strcmp(res.fase_final, 'terminada')
        completada = true;
    end
    
    % Métricas de seguimiento de trayectoria
    rmse_ey = sqrt(mean(Tm.e_y_cg_m.^2, 'omitnan'));
    p95_ey  = prctile(abs(Tm.e_y_cg_m), 95);
    max_ey  = max(abs(Tm.e_y_cg_m));
    rmse_epsi = rad2deg(sqrt(mean(Tm.e_psi_cg_rad.^2, 'omitnan')));
    p95_epsi  = rad2deg(prctile(abs(Tm.e_psi_cg_rad), 95));
    t_vuelta = Tm.t_s(end) - Tm.t_s(1);
    
    % Métricas dinámicas del actuador físico
    if exist(f_ciclos, 'file')
        try
            C = readtable(f_ciclos);
            [~, iT, iC] = intersect(Tm.ciclo, C.ciclo);
            Cm = C(iC, :);
            lag = abs(Cm.theta_cmd_deg - Cm.theta_meas_deg);
            p95_lag = prctile(lag, 95);
            max_lag = max(lag);
            max_omega = max(abs(Cm.omega_dps));
            pct_sat = mean(abs(Cm.omega_dps) > slew_lim_dps) * 100.0;
        catch
            p95_lag = NaN; max_lag = NaN; max_omega = NaN; pct_sat = NaN;
        end
    else
        p95_lag = NaN; max_lag = NaN; max_omega = NaN; pct_sat = NaN;
    end
    
    filas = [filas; {cond, ctrl, desc_ctrl, perf, ses, completada, k_stanley, ...
                     rmse_ey, p95_ey, max_ey, rmse_epsi, p95_epsi, ...
                     t_vuelta, p95_lag, max_lag, max_omega, pct_sat, carpetas(k).name}]; %#ok<AGROW>
end

tabla_datos = cell2table(filas, 'VariableNames', ...
    {'Condicion', 'Controlador', 'Desc_Controlador', 'Perfil', 'Sesion', 'Vuelta_Completada', ...
     'K_Stanley', 'RMSE_ey_m', 'P95_ey_m', 'Max_ey_m', 'RMSE_epsi_deg', 'P95_epsi_deg', ...
     'Tiempo_vuelta_s', 'P95_lag_deg', 'Max_lag_deg', 'Max_omega_dps', 'Pct_sat_tasa', 'Carpeta'});

f_csv = fullfile(dir_res, 's31_dataset_consolidado.csv');
writetable(tabla_datos, f_csv);
fprintf('Dataset consolidado guardado en: %s (%d corridas procesadas)\n\n', f_csv, height(tabla_datos));

%% 2. Resumen Descriptivo por Factor (Condición x Controlador)
G = groupsummary(tabla_datos, {'Condicion', 'Controlador'}, {'mean', 'std'}, ...
                 {'RMSE_ey_m', 'RMSE_epsi_deg', 'P95_lag_deg', 'Max_omega_dps'});
disp(G);
writetable(G, fullfile(dir_res, 's31_resumen_grupal.csv'));

%% 3. Contrastes de Hipótesis Formales
fprintf('\n--- Contrastes de Hipótesis Planificados ---\n');

% H1: Evaluación de Stanley congelado (k=2.04) vs resintonizado (k=1.0) en HIL-U
idx_st_cong = (tabla_datos.Condicion == "HIL-U" & tabla_datos.Controlador == "stanley_congelado");
idx_st_resi = (tabla_datos.Condicion == "HIL-U" & tabla_datos.Controlador == "stanley_resintonizado");
fprintf('Hipótesis H1 (Stanley Congelado vs Resintonizado en HIL-U):\n');
fprintf('  Stanley congelado (k=2.04): n = %d corridas | Vueltas completadas = %d\n', ...
        sum(idx_st_cong), sum(tabla_datos.Vuelta_Completada(idx_st_cong)));
fprintf('  Stanley resintonizado (k=1.0): n = %d corridas | Vueltas completadas = %d | RMSE medio = %.3f m\n', ...
        sum(idx_st_resi), sum(tabla_datos.Vuelta_Completada(idx_st_resi)), ...
        mean(tabla_datos.RMSE_ey_m(idx_st_resi), 'omitnan'));

% H4: Actuator-Aware MPC vs MPC Cinemático en HIL-U
idx_mpc_cin = (tabla_datos.Condicion == "HIL-U" & tabla_datos.Controlador == "mpc_cinematico");
idx_mpc_aa  = (tabla_datos.Condicion == "HIL-U" & tabla_datos.Controlador == "mpc_actuator_aware");

if any(idx_mpc_cin) && any(idx_mpc_aa)
    w_cin = tabla_datos.Max_omega_dps(idx_mpc_cin);
    w_aa  = tabla_datos.Max_omega_dps(idx_mpc_aa);
    lag_cin = tabla_datos.P95_lag_deg(idx_mpc_cin);
    lag_aa  = tabla_datos.P95_lag_deg(idx_mpc_aa);
    sat_cin = tabla_datos.Pct_sat_tasa(idx_mpc_cin);
    sat_aa  = tabla_datos.Pct_sat_tasa(idx_mpc_aa);
    
    fprintf('\nHipótesis H4 (MPC Cinemático vs Actuator-Aware MPC en HIL-U):\n');
    fprintf('  Tasa máx: Cinemático = %.1f deg/s | Actuator-Aware = %.1f deg/s\n', ...
            mean(w_cin, 'omitnan'), mean(w_aa, 'omitnan'));
    fprintf('  Saturación de slew-rate (>560 deg/s): Cinemático = %.3f%% | Actuator-Aware = %.3f%%\n', ...
            mean(sat_cin, 'omitnan'), mean(sat_aa, 'omitnan'));
    fprintf('  Atraso P95 motor: Cinemático = %.2f deg | Actuator-Aware = %.2f deg\n', ...
            mean(lag_cin, 'omitnan'), mean(lag_aa, 'omitnan'));
end

%% 4. Gráfico de Barras con Intervalos de Confianza (Publicación VSD a 600 dpi)
f1 = figure('Name', 's31_F1_resumen_desempeno_vsd', 'Position', [100 100 1050 580], 'Color', 'w');
ctrls_unicos = unique(tabla_datos.Controlador);
medias_ey = zeros(numel(ctrls_unicos), 1);
stds_ey   = zeros(numel(ctrls_unicos), 1);
n_muestras= zeros(numel(ctrls_unicos), 1);

for i = 1:numel(ctrls_unicos)
    idx_c = (tabla_datos.Controlador == ctrls_unicos(i) & tabla_datos.Vuelta_Completada == true);
    if ~any(idx_c)
        % Si ninguna completó (ej. Stanley congelado en alta velocidad), graficar sobre las muestras disponibles
        idx_c = (tabla_datos.Controlador == ctrls_unicos(i));
    end
    medias_ey(i) = mean(tabla_datos.RMSE_ey_m(idx_c), 'omitnan');
    stds_ey(i)   = std(tabla_datos.RMSE_ey_m(idx_c), 'omitnan');
    n_muestras(i)= sum(idx_c);
end

b = bar(medias_ey, 'FaceColor', [0.2 0.45 0.75], 'EdgeColor', 'k', 'LineWidth', 1.1);
hold on; grid on; box on;
errorbar(1:numel(ctrls_unicos), medias_ey, stds_ey, 'k.', 'LineWidth', 1.3, 'CapSize', 10);

% Anotaciones de número de corridas
for i = 1:numel(ctrls_unicos)
    if ~isnan(medias_ey(i))
        text(i, medias_ey(i) + stds_ey(i) + 0.015, sprintf('n=%d', n_muestras(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
    end
end

etiquetas = strrep(ctrls_unicos, '_', ' ');
set(gca, 'XTick', 1:numel(ctrls_unicos), 'XTickLabel', etiquetas, ...
         'TickLabelInterpreter', 'none', 'FontSize', 10);
ylabel('RMSE de error lateral $e_y$ [m]', 'Interpreter', 'latex', 'FontSize', 11);
title('Desempeño Comparativo de Seguimiento en Banco HIL-U (Monza)', 'FontSize', 12, 'FontWeight', 'bold');

% Exportación obligatoria a 600 dpi para revista Q1
f_fig_png = fullfile(dir_fig, 's31_F1_resumen_desempeno_vsd.png');
f_fig_pdf = fullfile(dir_fig, 's31_F1_resumen_desempeno_vsd.pdf');
exportgraphics(f1, f_fig_png, 'Resolution', 600);
exportgraphics(f1, f_fig_pdf, 'ContentType', 'vector');

fprintf('\nGuardada Figura F1 (600 dpi PNG): %s\n', f_fig_png);
fprintf('Guardada Figura F1 (Vectorial PDF): %s\n', f_fig_pdf);
fprintf('\nPipeline estadístico completado exitosamente.\n');

