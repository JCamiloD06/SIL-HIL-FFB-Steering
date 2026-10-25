%% s37_analisis_campana_hill.m
% Analisis integral de la campana HIL-L (Sesiones 1 a 10) y comparativa
% sistematica frente a la linea base HIL-U (Sesiones 1 a 10).
%
% Evalua cuantitativamente el impacto de la carga fisica de force feedback
% (torque de autoalineacion tau_load proveniente de Assetto Corsa via USB HID)
% sobre el desempeno, la estabilidad y la realizabilidad de los cuatro
% controladores laterales (Pure Pursuit, Stanley resintonizado, MPC Cinematico
% y Actuator-Aware MPC) en el Autodromo Nazionale Monza.
%
% Salidas:
%   - results/primary/campana_hill/s37_tabla_campana_hill_s1_a_s10.csv
%   - results/primary/campana_hill/s37_tabla_campana_hill_completa.csv
%   - results/primary/campana_hill/s37_comparativa_hilu_vs_hill.csv
%   - figures/final/campana_hill/s37_F1_metricas_globales_hill_completa.png (.pdf)
%   - figures/final/campana_hill/s37_F1_metricas_globales_hill_s1_a_s7.png (.pdf)
%   - figures/final/campana_hill/s37_F2_comparativa_hilu_vs_hill.png (.pdf)
%   - figures/final/campana_hill/s37_F3_chicane1_dinamica_hill.png (.pdf)
%
% Protocolo: AGENTS.md §17 (MATLAB para analisis cuantitativo, D-022) y §19.

clearvars; close all; clc;

raiz = fileparts(fileparts(mfilename('fullpath')));
if isempty(raiz), raiz = pwd; end

dir_datos = fullfile(raiz, 'data', 'raw', 'trialsteer');
dir_res = fullfile(raiz, 'results', 'primary', 'campana_hill');
dir_fig = fullfile(raiz, 'figures', 'final', 'campana_hill');

for d = {dir_res, dir_fig}
    if ~exist(d{1}, 'dir'), mkdir(d{1}); end
end

fprintf('========================================================================\n');
fprintf('  s37_analisis_campana_hill: Analisis Integral HIL-L y Comparativa HIL-U\n');
fprintf('========================================================================\n\n');

kw = 19.4888;          % deg volante / deg rueda (rango 900 deg / rueda 0.403 rad)
slew_lim_dps = 560.0;   % limite fisico de velocidad angular del banco (deg/s)

%% 1. Escanear y procesar corridas de HIL-L (Sesiones 1 a 10)
pat_hill = dir(fullfile(dir_datos, 'HILL_campana_s*'));
fprintf('Total de carpetas HIL-L encontradas: %d\n', numel(pat_hill));

filas_hill = {};

for k = 1:numel(pat_hill)
    c_dir = fullfile(dir_datos, pat_hill(k).name);
    f_manif = fullfile(c_dir, 'manifiesto.json');
    f_telem = fullfile(c_dir, 'telemetria.csv');
    f_ciclos = fullfile(c_dir, 'odrive_ciclos.csv');
    f_espejo = fullfile(c_dir, 'odrive_espejo.csv');
    
    if ~exist(f_manif, 'file') || ~exist(f_telem, 'file'), continue; end
    
    try
        manif = jsondecode(fileread(f_manif));
    catch
        continue;
    end
    
    meta = manif.meta;
    res = manif.resumen;
    ses = meta.sesion;
    ctrl = string(meta.controlador);
    perf = string(meta.perfil);
    id_plan = string(meta.id_plan);
    
    completada = false;
    if isfield(res, 'vuelta_completada') && ~isempty(res.vuelta_completada)
        completada = logical(res.vuelta_completada);
    end
    
    t_vuelta = NaN;
    if isfield(res, 'tiempo_vuelta_s') && ~isempty(res.tiempo_vuelta_s)
        t_vuelta = double(res.tiempo_vuelta_s);
    end
    
    ruedas_fuera = 0;
    if isfield(res, 'max_ruedas_fuera') && ~isempty(res.max_ruedas_fuera)
        ruedas_fuera = double(res.max_ruedas_fuera);
    end
    
    try
        T = readtable(f_telem);
    catch
        continue;
    end
    
    if ismember('fase_vuelta', T.Properties.VariableNames)
        idx_m = strcmp(string(T.fase_vuelta), "medida");
        if ~any(idx_m), idx_m = true(height(T), 1); end
    else
        idx_m = true(height(T), 1);
    end
    Tm = T(idx_m, :);
    
    rmse_ey = NaN; max_ey = NaN; rmse_epsi = NaN; max_epsi = NaN;
    if height(Tm) >= 10
        ey = Tm.e_y_cg_m;
        epsi = Tm.e_psi_cg_rad * (180 / pi);
        rmse_ey = sqrt(mean(ey.^2, 'omitnan'));
        max_ey = max(abs(ey));
        rmse_epsi = sqrt(mean(epsi.^2, 'omitnan'));
        max_epsi = max(abs(epsi));
    end
    
    p95_lag = NaN; mean_lag = NaN; max_omega = NaN; pct_sat_slew = NaN;
    if exist(f_ciclos, 'file')
        try
            To = readtable(f_ciclos);
            if ismember('theta_cmd_deg', To.Properties.VariableNames) && ...
               ismember('theta_meas_deg', To.Properties.VariableNames)
                d_lag = abs(To.theta_cmd_deg - To.theta_meas_deg);
                p95_lag = prctile(d_lag, 95);
                mean_lag = mean(d_lag, 'omitnan');
                
                if ismember('omega_dps', To.Properties.VariableNames)
                    w = abs(To.omega_dps);
                    max_omega = max(w);
                    pct_sat_slew = sum(w >= slew_lim_dps) / numel(w) * 100.0;
                end
            end
        catch
        end
    end
    
    mean_tau_load = NaN; p95_tau_load = NaN; max_tau_load = NaN;
    if exist(f_espejo, 'file')
        try
            Te = readtable(f_espejo);
            if ismember('tau_carga_nm', Te.Properties.VariableNames)
                tau_abs = abs(Te.tau_carga_nm);
                mean_tau_load = mean(tau_abs, 'omitnan');
                p95_tau_load = prctile(tau_abs, 95);
                max_tau_load = max(tau_abs);
            end
        catch
        end
    end
    
    filas_hill = [filas_hill; {string(pat_hill(k).name), ses, id_plan, ctrl, perf, completada, ...
                               t_vuelta, rmse_ey, max_ey, rmse_epsi, max_epsi, ...
                               p95_lag, mean_lag, max_omega, pct_sat_slew, ...
                               mean_tau_load, p95_tau_load, max_tau_load, ...
                               ruedas_fuera, height(Tm)}];
end

nombres_col_hill = {'carpeta', 'sesion', 'id_plan', 'controlador', 'perfil', 'vuelta_completada', ...
                    'tiempo_vuelta_s', 'rmse_ey_m', 'max_ey_m', 'rmse_epsi_deg', 'max_epsi_deg', ...
                    'p95_lag_deg', 'mean_lag_deg', 'max_omega_dps', 'pct_sat_slew', ...
                    'mean_tau_load_nm', 'p95_tau_load_nm', 'max_tau_load_nm', ...
                    'ruedas_fuera', 'n_muestras'};

T_hill = cell2table(filas_hill, 'VariableNames', nombres_col_hill);
ruta_csv_hill = fullfile(dir_res, 's37_tabla_campana_hill_s1_a_s10.csv');
writetable(T_hill, ruta_csv_hill);
writetable(T_hill, fullfile(dir_res, 's37_tabla_campana_hill_completa.csv'));
writetable(T_hill, fullfile(dir_res, 's37_tabla_campana_hill_s1_a_s7.csv'));
fprintf('Tabla HIL-L guardada: %s (%d corridas)\n', ruta_csv_hill, height(T_hill));

%% 2. Escanear y procesar corridas de HIL-U (Sesiones 1 a 10)
pat_hilu = dir(fullfile(dir_datos, 'HILU_campana_s*'));
fprintf('Total de carpetas HIL-U encontradas: %d\n', numel(pat_hilu));

filas_hilu = {};
for k = 1:numel(pat_hilu)
    c_dir = fullfile(dir_datos, pat_hilu(k).name);
    f_manif = fullfile(c_dir, 'manifiesto.json');
    f_telem = fullfile(c_dir, 'telemetria.csv');
    f_ciclos = fullfile(c_dir, 'odrive_ciclos.csv');
    
    if ~exist(f_manif, 'file') || ~exist(f_telem, 'file'), continue; end
    
    try
        manif = jsondecode(fileread(f_manif));
    catch
        continue;
    end
    
    meta = manif.meta;
    res = manif.resumen;
    ses = meta.sesion;
    ctrl = string(meta.controlador);
    perf = string(meta.perfil);
    id_plan = string(meta.id_plan);
    
    completada = false;
    if isfield(res, 'vuelta_completada') && ~isempty(res.vuelta_completada)
        completada = logical(res.vuelta_completada);
    end
    
    t_vuelta = NaN;
    if isfield(res, 'tiempo_vuelta_s') && ~isempty(res.tiempo_vuelta_s)
        t_vuelta = double(res.tiempo_vuelta_s);
    end
    
    ruedas_fuera = 0;
    if isfield(res, 'max_ruedas_fuera') && ~isempty(res.max_ruedas_fuera)
        ruedas_fuera = double(res.max_ruedas_fuera);
    end
    
    try
        T = readtable(f_telem);
    catch
        continue;
    end
    
    if ismember('fase_vuelta', T.Properties.VariableNames)
        idx_m = strcmp(string(T.fase_vuelta), "medida");
        if ~any(idx_m), idx_m = true(height(T), 1); end
    else
        idx_m = true(height(T), 1);
    end
    Tm = T(idx_m, :);
    
    rmse_ey = NaN; max_ey = NaN; rmse_epsi = NaN; max_epsi = NaN;
    if height(Tm) >= 10
        ey = Tm.e_y_cg_m;
        epsi = Tm.e_psi_cg_rad * (180 / pi);
        rmse_ey = sqrt(mean(ey.^2, 'omitnan'));
        max_ey = max(abs(ey));
        rmse_epsi = sqrt(mean(epsi.^2, 'omitnan'));
        max_epsi = max(abs(epsi));
    end
    
    p95_lag = NaN; mean_lag = NaN; max_omega = NaN; pct_sat_slew = NaN;
    if exist(f_ciclos, 'file')
        try
            To = readtable(f_ciclos);
            if ismember('theta_cmd_deg', To.Properties.VariableNames) && ...
               ismember('theta_meas_deg', To.Properties.VariableNames)
                d_lag = abs(To.theta_cmd_deg - To.theta_meas_deg);
                p95_lag = prctile(d_lag, 95);
                mean_lag = mean(d_lag, 'omitnan');
                
                if ismember('omega_dps', To.Properties.VariableNames)
                    w = abs(To.omega_dps);
                    max_omega = max(w);
                    pct_sat_slew = sum(w >= slew_lim_dps) / numel(w) * 100.0;
                end
            end
        catch
        end
    end
    
    filas_hilu = [filas_hilu; {string(pat_hilu(k).name), ses, id_plan, ctrl, perf, completada, ...
                               t_vuelta, rmse_ey, max_ey, rmse_epsi, max_epsi, ...
                               p95_lag, mean_lag, max_omega, pct_sat_slew, ...
                               ruedas_fuera, height(Tm)}];
end

nombres_col_hilu = {'carpeta', 'sesion', 'id_plan', 'controlador', 'perfil', 'vuelta_completada', ...
                    'tiempo_vuelta_s', 'rmse_ey_m', 'max_ey_m', 'rmse_epsi_deg', 'max_epsi_deg', ...
                    'p95_lag_deg', 'mean_lag_deg', 'max_omega_dps', 'pct_sat_slew', ...
                    'ruedas_fuera', 'n_muestras'};

T_hilu = cell2table(filas_hilu, 'VariableNames', nombres_col_hilu);

%% 3. Tabla Comparativa Sistematica HIL-U vs HIL-L
ctrl_list = ["pure_pursuit", "stanley", "mpc_cinematico", "mpc_actuator_aware"];
perf_list = ["conservador", "nominal", "rapido"];
nombres_display = containers.Map(...
    {'pure_pursuit', 'stanley', 'mpc_cinematico', 'mpc_actuator_aware'}, ...
    {'Pure Pursuit', 'Stanley (k=1.0)', 'MPC Cinematico', 'Actuator-Aware MPC'});

filas_comp = {};

fprintf('\n========================================================================================================\n');
fprintf('  TABLA COMPARATIVA SISTEMATICA: HIL-U (SIN CARGA) vs HIL-L (CON CARGA FFB)\n');
fprintf('========================================================================================================\n');
fprintf('%-18s | %-11s | %-11s | %-11s | %-12s | %-12s | %-10s | %-10s\n', ...
        'Controlador', 'Perfil', 'OK HIL-U', 'OK HIL-L', 'ey HIL-U [m]', 'ey HIL-L [m]', 'Delta ey (%)', 'p95 tau [Nm]');
fprintf('%s\n', repmat('-', 1, 104));

for c_idx = 1:numel(ctrl_list)
    c_name = ctrl_list(c_idx);
    for p_idx = 1:numel(perf_list)
        p_name = perf_list(p_idx);
        
        sub_u = T_hilu(T_hilu.controlador == c_name & T_hilu.perfil == p_name, :);
        % Deduplicar por id_plan
        [~, iu_u] = unique(sub_u.id_plan, 'last');
        sub_u = sub_u(iu_u, :);
        n_ok_u = sum(sub_u.vuelta_completada);
        n_tot_u = height(sub_u);
        sub_u_ok = sub_u(sub_u.vuelta_completada, :);
        m_ey_u = mean(sub_u_ok.rmse_ey_m, 'omitnan');
        std_ey_u = std(sub_u_ok.rmse_ey_m, 'omitnan');
        m_lag_u = mean(sub_u_ok.p95_lag_deg, 'omitnan');
        
        sub_l = T_hill(T_hill.controlador == c_name & T_hill.perfil == p_name, :);
        [~, iu_l] = unique(sub_l.id_plan, 'last');
        sub_l = sub_l(iu_l, :);
        n_ok_l = sum(sub_l.vuelta_completada);
        n_tot_l = height(sub_l);
        sub_l_ok = sub_l(sub_l.vuelta_completada, :);
        m_ey_l = mean(sub_l_ok.rmse_ey_m, 'omitnan');
        std_ey_l = std(sub_l_ok.rmse_ey_m, 'omitnan');
        m_lag_l = mean(sub_l_ok.p95_lag_deg, 'omitnan');
        m_tau_l = mean(sub_l.p95_tau_load_nm, 'omitnan');
        
        delta_ey_pct = ((m_ey_l - m_ey_u) / m_ey_u) * 100;
        delta_lag_deg = m_lag_l - m_lag_u;
        
        str_ok_u = sprintf('%d/%d', n_ok_u, n_tot_u);
        str_ok_l = sprintf('%d/%d', n_ok_l, n_tot_l);
        
        fprintf('%-18s | %-11s | %-11s | %-11s | %6.3f±%5.3f | %6.3f±%5.3f | %+9.1f%% | %10.3f\n', ...
                nombres_display(char(c_name)), p_name, str_ok_u, str_ok_l, ...
                m_ey_u, std_ey_u, m_ey_l, std_ey_l, delta_ey_pct, m_tau_l);
            
        filas_comp = [filas_comp; {c_name, p_name, n_ok_u, n_tot_u, n_ok_l, n_tot_l, ...
                                   m_ey_u, std_ey_u, m_ey_l, std_ey_l, delta_ey_pct, ...
                                   m_lag_u, m_lag_l, delta_lag_deg, m_tau_l}];
    end
    fprintf('%s\n', repmat('-', 1, 104));
end

nombres_col_comp = {'controlador', 'perfil', 'ok_hilu', 'tot_hilu', 'ok_hill', 'tot_hill', ...
                    'rmse_ey_hilu_m', 'std_ey_hilu_m', 'rmse_ey_hill_m', 'std_ey_hill_m', 'delta_ey_pct', ...
                    'p95_lag_hilu_deg', 'p95_lag_hill_deg', 'delta_lag_deg', 'p95_tau_load_nm'};
T_comp = cell2table(filas_comp, 'VariableNames', nombres_col_comp);
ruta_csv_comp = fullfile(dir_res, 's37_comparativa_hilu_vs_hill.csv');
writetable(T_comp, ruta_csv_comp);
fprintf('Tabla comparativa guardada: %s\n\n', ruta_csv_comp);

%% 4. Figura F1: Metricas Globales de la Campana HIL-L (600 dpi)
fig1 = figure('Position', [80, 80, 1200, 750], 'Color', 'w');

cols = [0.20 0.45 0.85;   % Pure Pursuit (azul)
        0.85 0.35 0.10;   % Stanley (naranja)
        0.70 0.15 0.20;   % MPC Cinematico (rojo oscuro)
        0.12 0.65 0.30];  % Actuator-Aware MPC (verde)

% (a) Tasa de Culminacion (%) en HIL-L
subplot(2, 2, 1); hold on; grid on; box on;
tasas_hill = zeros(4, 3);
for c = 1:4
    for p = 1:3
        sub = T_comp(T_comp.controlador == ctrl_list(c) & T_comp.perfil == perf_list(p), :);
        if ~isempty(sub)
            tasas_hill(c, p) = (sub.ok_hill(1) / sub.tot_hill(1)) * 100;
        end
    end
end
b1 = bar(tasas_hill', 'grouped');
for c = 1:4, b1(c).FaceColor = cols(c, :); end
set(gca, 'XTickLabel', {'Conservador (f=0.8)', 'Nominal (f=0.9)', 'Rapido (f=1.0)'}, ...
         'FontSize', 10, 'FontName', 'Arial');
ylabel('Tasa de Culminacion (%)', 'FontSize', 11, 'FontWeight', 'bold');
title('(a) Tasa de Culminacion en HIL-L (Carga Activa)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0 115]); yline(100, 'k--', 'LineWidth', 0.8);
legend({'Pure Pursuit', 'Stanley (k=1.0)', 'MPC Cinematico', 'Actuator-Aware MPC'}, ...
       'Location', 'southwest', 'FontSize', 9);

% (b) Precision Lateral: RMSE(e_y) con barras de error inter-sesion
subplot(2, 2, 2); hold on; grid on; box on;
media_ey = zeros(4, 3);
std_ey = zeros(4, 3);
for c = 1:4
    for p = 1:3
        sub = T_comp(T_comp.controlador == ctrl_list(c) & T_comp.perfil == perf_list(p), :);
        if ~isempty(sub) && sub.ok_hill(1) > 0
            media_ey(c, p) = sub.rmse_ey_hill_m(1);
            std_ey(c, p) = sub.std_ey_hill_m(1);
        else
            media_ey(c, p) = NaN; std_ey(c, p) = NaN;
        end
    end
end
b2 = bar(media_ey', 'grouped');
for c = 1:4, b2(c).FaceColor = cols(c, :); end

nbars = size(media_ey, 1);
ngroups = size(media_ey, 2);
groupwidth = min(0.8, nbars/(nbars + 1.5));
for c = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*c - 1) * groupwidth / (2*nbars);
    errorbar(x, media_ey(c, :), std_ey(c, :), 'k', 'linestyle', 'none', 'LineWidth', 1.0, 'CapSize', 4);
end
set(gca, 'XTickLabel', {'Conservador', 'Nominal', 'Rapido'}, 'FontSize', 10, 'FontName', 'Arial');
ylabel('RMSE(e_y) [m]', 'FontSize', 11, 'FontWeight', 'bold');
title('(b) Error Lateral en HIL-L (Media ± \sigma inter-sesion)', 'FontSize', 12, 'FontWeight', 'bold');
legend({'Pure Pursuit', 'Stanley (k=1.0)', 'MPC Cinematico', 'Actuator-Aware MPC'}, ...
       'Location', 'northwest', 'FontSize', 9);

% (c) Torque Fisico de Carga FFB aplicado: p95(|tau_load|) [N*m]
subplot(2, 2, 3); hold on; grid on; box on;
media_tau = zeros(4, 3);
std_tau = zeros(4, 3);
for c = 1:4
    for p = 1:3
        sub = T_hill(T_hill.controlador == ctrl_list(c) & T_hill.perfil == perf_list(p), :);
        if ~isempty(sub)
            media_tau(c, p) = mean(sub.p95_tau_load_nm, 'omitnan');
            std_tau(c, p) = std(sub.p95_tau_load_nm, 'omitnan');
        end
    end
end
b3 = bar(media_tau', 'grouped');
for c = 1:4, b3(c).FaceColor = cols(c, :); end
for c = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*c - 1) * groupwidth / (2*nbars);
    errorbar(x, media_tau(c, :), std_tau(c, :), 'k', 'linestyle', 'none', 'LineWidth', 1.0, 'CapSize', 4);
end
set(gca, 'XTickLabel', {'Conservador', 'Nominal', 'Rapido'}, 'FontSize', 10, 'FontName', 'Arial');
ylabel('p95 |\tau_{load}| [N\cdot m]', 'FontSize', 11, 'FontWeight', 'bold');
title('(c) Carga Fisica FFB Demandada en el Volante', 'FontSize', 12, 'FontWeight', 'bold');
yline(1.0, 'r--', 'Tope de Seguridad (1.0 N\cdot m)', 'LineWidth', 1.0, 'LabelHorizontalAlignment', 'left');

% (d) Estabilidad y Repetibilidad del Tiempo de Vuelta
subplot(2, 2, 4); hold on; grid on; box on;
markers = {'o-', 's-', '^-', 'd-'};
max_ses = max(T_hill.sesion);
ses_vec = 1:max_ses;
for c = 1:4
    t_ses_nom = NaN(1, max_ses);
    for s = 1:max_ses
        sub = T_hill(T_hill.controlador == ctrl_list(c) & T_hill.perfil == "nominal" & ...
                     T_hill.sesion == s & T_hill.vuelta_completada, :);
        if ~isempty(sub)
            t_ses_nom(s) = mean(sub.tiempo_vuelta_s, 'omitnan');
        end
    end
    plot(ses_vec, t_ses_nom, markers{c}, 'Color', cols(c, :), ...
         'MarkerFaceColor', cols(c, :), 'LineWidth', 1.5, 'MarkerSize', 6);
end
set(gca, 'XTick', 1:max_ses, 'FontSize', 10, 'FontName', 'Arial');
xlabel('Numero de Sesion (HIL-L)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Tiempo de Vuelta Nominal [s]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('(d) Repetibilidad Inter-Sesion (Perfil Nominal, S1 a S%d)', max_ses), 'FontSize', 12, 'FontWeight', 'bold');
ylim([188.5 189.5]);
legend({'Pure Pursuit', 'Stanley (k=1.0)', 'MPC Cinematico', 'Actuator-Aware MPC'}, ...
       'Location', 'northeast', 'FontSize', 9);

f1_png = fullfile(dir_fig, 's37_F1_metricas_globales_hill_completa.png');
f1_pdf = fullfile(dir_fig, 's37_F1_metricas_globales_hill_completa.pdf');
exportgraphics(fig1, f1_png, 'Resolution', 600);
exportgraphics(fig1, f1_pdf, 'ContentType', 'vector');
exportgraphics(fig1, fullfile(dir_fig, 's37_F1_metricas_globales_hill_s1_a_s7.png'), 'Resolution', 600);
exportgraphics(fig1, fullfile(dir_fig, 's37_F1_metricas_globales_hill_s1_a_s7.pdf'), 'ContentType', 'vector');
fprintf('Figura F1 exportada:\n  - %s\n  - %s\n', f1_png, f1_pdf);

%% 5. Figura F2: Comparativa Directa HIL-U vs HIL-L
fig2 = figure('Position', [120, 120, 1200, 750], 'Color', 'w');

% (a) Degradacion Relativa de Precision Lateral: Delta RMSE(e_y) (%)
subplot(2, 2, 1); hold on; grid on; box on;
deg_ey_pct = zeros(4, 2);
for c = 1:4
    for p = 1:2
        sub_c = T_comp(T_comp.controlador == ctrl_list(c) & T_comp.perfil == perf_list(p), :);
        if ~isempty(sub_c)
            deg_ey_pct(c, p) = sub_c.delta_ey_pct(1);
        end
    end
end
b21 = bar(deg_ey_pct, 'grouped');
b21(1).FaceColor = [0.25 0.50 0.75];
b21(2).FaceColor = [0.85 0.40 0.20];
set(gca, 'XTickLabel', {'Pure Pursuit', 'Stanley (k=1.0)', 'MPC Cinem.', 'Actuator-Aware'}, ...
         'FontSize', 10, 'FontName', 'Arial');
ylabel('\Delta RMSE(e_y) [%]', 'FontSize', 11, 'FontWeight', 'bold');
title('(a) Degradacion de Precision Lateral por Carga FFB', 'FontSize', 12, 'FontWeight', 'bold');
legend({'Conservador (f=0.8)', 'Nominal (f=0.9)'}, 'Location', 'northwest', 'FontSize', 9);
yline(0, 'k-', 'LineWidth', 0.8);

% (b) Incremento de Atraso Angular Dinamico: p95 Lag HIL-U vs HIL-L
subplot(2, 2, 2); hold on; grid on; box on;
lag_comp = zeros(4, 2);
for c = 1:4
    sub_c = T_comp(T_comp.controlador == ctrl_list(c) & T_comp.perfil == "nominal", :);
    if ~isempty(sub_c)
        lag_comp(c, 1) = sub_c.p95_lag_hilu_deg(1);
        lag_comp(c, 2) = sub_c.p95_lag_hill_deg(1);
    end
end
b22 = bar(lag_comp, 'grouped');
b22(1).FaceColor = [0.60 0.60 0.65];
b22(2).FaceColor = [0.75 0.15 0.25];
set(gca, 'XTickLabel', {'Pure Pursuit', 'Stanley (k=1.0)', 'MPC Cinem.', 'Actuator-Aware'}, ...
         'FontSize', 10, 'FontName', 'Arial');
ylabel('p95 Lag Angular [^\circ volante]', 'FontSize', 11, 'FontWeight', 'bold');
title('(b) Desfase Dinamico del Volante: HIL-U vs HIL-L', 'FontSize', 12, 'FontWeight', 'bold');
legend({'HIL-U (Sin Carga)', 'HIL-L (Carga Activa)'}, 'Location', 'northwest', 'FontSize', 9);

% (c) Porcentaje de Saturacion de Slew-Rate (560 deg/s)
subplot(2, 2, 3); hold on; grid on; box on;
sat_comp = zeros(4, 2);
for c = 1:4
    sub_u = T_hilu(T_hilu.controlador == ctrl_list(c) & T_hilu.perfil == "nominal" & T_hilu.vuelta_completada, :);
    sub_l = T_hill(T_hill.controlador == ctrl_list(c) & T_hill.perfil == "nominal" & T_hill.vuelta_completada, :);
    sat_comp(c, 1) = mean(sub_u.pct_sat_slew, 'omitnan');
    sat_comp(c, 2) = mean(sub_l.pct_sat_slew, 'omitnan');
end
b23 = bar(sat_comp, 'grouped');
b23(1).FaceColor = [0.60 0.60 0.65];
b23(2).FaceColor = [0.15 0.65 0.40];
set(gca, 'XTickLabel', {'Pure Pursuit', 'Stanley (k=1.0)', 'MPC Cinem.', 'Actuator-Aware'}, ...
         'FontSize', 10, 'FontName', 'Arial');
ylabel('Saturacion de Tasa [% del tiempo]', 'FontSize', 11, 'FontWeight', 'bold');
title('(c) Saturacion de Slew-Rate (> 560 ^\circ/s)', 'FontSize', 12, 'FontWeight', 'bold');
legend({'HIL-U (Sin Carga)', 'HIL-L (Carga Activa)'}, 'Location', 'northeast', 'FontSize', 9);

% (d) Resiliencia en Perfil Rapido: Tasa de Culminacion HIL-U vs HIL-L
subplot(2, 2, 4); hold on; grid on; box on;
culm_comp = zeros(4, 2);
for c = 1:4
    sub_u = T_comp(T_comp.controlador == ctrl_list(c) & T_comp.perfil == "rapido", :);
    if ~isempty(sub_u)
        culm_comp(c, 1) = (sub_u.ok_hilu(1) / sub_u.tot_hilu(1)) * 100;
        culm_comp(c, 2) = (sub_u.ok_hill(1) / sub_u.tot_hill(1)) * 100;
    end
end
b24 = bar(culm_comp, 'grouped');
b24(1).FaceColor = [0.40 0.70 0.90];
b24(2).FaceColor = [0.90 0.25 0.25];
set(gca, 'XTickLabel', {'Pure Pursuit', 'Stanley (k=1.0)', 'MPC Cinem.', 'Actuator-Aware'}, ...
         'FontSize', 10, 'FontName', 'Arial');
ylabel('Culminacion en Perfil Rapido [%]', 'FontSize', 11, 'FontWeight', 'bold');
title('(d) Colapso de Viabilidad en Perfil Rapido', 'FontSize', 12, 'FontWeight', 'bold');
legend({'HIL-U (Sin Carga)', 'HIL-L (Carga Activa)'}, 'Location', 'northeast', 'FontSize', 9);
ylim([0 115]); yline(100, 'k--', 'LineWidth', 0.8);

f2_png = fullfile(dir_fig, 's37_F2_comparativa_hilu_vs_hill.png');
f2_pdf = fullfile(dir_fig, 's37_F2_comparativa_hilu_vs_hill.pdf');
exportgraphics(fig2, f2_png, 'Resolution', 600);
exportgraphics(fig2, f2_pdf, 'ContentType', 'vector');
fprintf('Figura F2 exportada:\n  - %s\n  - %s\n', f2_png, f2_pdf);

%% 6. Figura F3: Dinamica Transitoria en la Chicane 1 (Variante del Rettifilo, HIL-L)
fig3 = figure('Position', [150, 150, 1100, 700], 'Color', 'w');

corridas_f3 = containers.Map();
max_ses = max(T_hill.sesion);
for c = 1:4
    sub = T_hill(T_hill.controlador == ctrl_list(c) & T_hill.perfil == "nominal" & ...
                 T_hill.sesion == max_ses & T_hill.vuelta_completada, :);
    if ~isempty(sub)
        corridas_f3(char(ctrl_list(c))) = char(sub.carpeta(1));
    else
        sub_any = T_hill(T_hill.controlador == ctrl_list(c) & T_hill.perfil == "nominal" & ...
                         T_hill.vuelta_completada, :);
        if ~isempty(sub_any)
            corridas_f3(char(ctrl_list(c))) = char(sub_any.carpeta(1));
        end
    end
end

ax1 = subplot(3, 1, 1); hold on; grid on; box on;
ax2 = subplot(3, 1, 2); hold on; grid on; box on;
ax3 = subplot(3, 1, 3); hold on; grid on; box on;

for c = 1:4
    c_key = char(ctrl_list(c));
    if ~isKey(corridas_f3, c_key), continue; end
    c_folder = fullfile(dir_datos, corridas_f3(c_key));
    
    f_t = fullfile(c_folder, 'telemetria.csv');
    f_e = fullfile(c_folder, 'odrive_espejo.csv');
    
    if ~exist(f_t, 'file') || ~exist(f_e, 'file'), continue; end
    
    Tt = readtable(f_t);
    Te = readtable(f_e);
    
    if ismember('s_cg_m', Tt.Properties.VariableNames)
        idx_ch = Tt.s_cg_m >= 850 & Tt.s_cg_m <= 1150;
        dist_s = Tt.s_cg_m(idx_ch);
    elseif ismember('s_m', Tt.Properties.VariableNames)
        idx_ch = Tt.s_m >= 850 & Tt.s_m <= 1150;
        dist_s = Tt.s_m(idx_ch);
    else
        idx_ch = true(height(Tt), 1);
        dist_s = (1:height(Tt))';
    end
    Ttch = Tt(idx_ch, :);
    
    if height(Ttch) < 5, continue; end
    
    theta_w = Ttch.delta_aplicado_rad * (180 / pi) * kw;
    ey_val = Ttch.e_y_cg_m;
    
    tau_interp = zeros(size(dist_s));
    if ismember('t_perf_s', Te.Properties.VariableNames) && ismember('tau_carga_nm', Te.Properties.VariableNames)
        t_telem = Ttch.t_s;
        tau_interp = interp1(Te.t_perf_s - Te.t_perf_s(1), Te.tau_carga_nm, ...
                             t_telem - t_telem(1), 'linear', 0);
    end
    
    plot(ax1, dist_s, theta_w, 'Color', cols(c, :), 'LineWidth', 1.4, ...
         'DisplayName', char(nombres_display(c_key)));
    plot(ax2, dist_s, tau_interp, 'Color', cols(c, :), 'LineWidth', 1.4, ...
         'DisplayName', char(nombres_display(c_key)));
    plot(ax3, dist_s, ey_val, 'Color', cols(c, :), 'LineWidth', 1.4, ...
         'DisplayName', char(nombres_display(c_key)));
end

set(ax1, 'FontSize', 10, 'FontName', 'Arial');
ylabel(ax1, '\theta_{volante} [^\circ]', 'FontSize', 11, 'FontWeight', 'bold');
title(ax1, 'Respuesta Dinamica en la Variante del Rettifilo (Chicane 1, HIL-L Nominal)', ...
      'FontSize', 12, 'FontWeight', 'bold');
legend(ax1, 'Location', 'northeast', 'FontSize', 9);

set(ax2, 'FontSize', 10, 'FontName', 'Arial');
ylabel(ax2, '\tau_{load} FFB [N\cdot m]', 'FontSize', 11, 'FontWeight', 'bold');
title(ax2, 'Torque de Autoalineacion Resistivo FFB', 'FontSize', 11, 'FontWeight', 'bold');

set(ax3, 'FontSize', 10, 'FontName', 'Arial');
xlabel(ax3, 'Distancia de Pista s [m]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax3, 'Error Lateral e_y [m]', 'FontSize', 11, 'FontWeight', 'bold');
title(ax3, 'Error de Seguimiento Lateral de Trayectoria', 'FontSize', 11, 'FontWeight', 'bold');
yline(ax3, 0, 'k--', 'LineWidth', 0.8);
linkaxes([ax1, ax2, ax3], 'x');
xlim(ax1, [880 1120]);

f3_png = fullfile(dir_fig, 's37_F3_chicane1_dinamica_hill.png');
f3_pdf = fullfile(dir_fig, 's37_F3_chicane1_dinamica_hill.pdf');
exportgraphics(fig3, f3_png, 'Resolution', 600);
exportgraphics(fig3, f3_pdf, 'ContentType', 'vector');
fprintf('Figura F3 exportada:\n  - %s\n  - %s\n\n', f3_png, f3_pdf);

fprintf('========================================================================\n');
fprintf('  s37_analisis_campana_hill completado con exito.\n');
fprintf('========================================================================\n');
