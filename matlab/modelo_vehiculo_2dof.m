%% modelo_vehiculo_2dof.m
% Linear 2-DOF bicycle model simulation and understeer gradient Kus calculation.
% Author: Juan Camilo Diaz-Lopez
% Department of Mechanical and Mechatronics Engineering
% Universidad Nacional de Colombia, Sede La Paz
% Research Paper: Physical Steering Dynamics and Force-Feedback Loading
%                 in Autonomous Path Tracking (Ingenieria e Investigacion)
%
if ~exist('raiz', 'var')
    raiz = fileparts(fileparts(mfilename('fullpath')));
end
if ~exist('carpetas', 'var')
    d = [dir(fullfile(raiz, 'data', 'raw', 'p00_referencia', 'campana_*')); ...
         dir(fullfile(raiz, 'data', 'raw', 'trialsteer', 'SIL_*')); ...
         dir(fullfile(raiz, 'data', 'raw', 'trialsteer', 'HILU_*'))];
    d = d([d.isdir]);
    carpetas = fullfile({d.folder}, {d.name});
end
assert(~isempty(carpetas), 'No hay corridas');

dres = fullfile(raiz, 'results', 'exploratory', 'cadena_vehicular');
dfig = fullfile(raiz, 'figures', 'draft', 'cadena_vehicular');
dlog = fullfile(raiz, 'logs');
for dd = {dres, dfig, dlog}
    if ~exist(dd{1}, 'dir'), mkdir(dd{1}); end
end
flog = fullfile(dlog, 's15_cadena_vehicular.log');
if isfile(flog), delete(flog); end
diary(flog);
fprintf('s15_cadena_vehicular.m  %s  MATLAB %s\n%d corridas\n', ...
    char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), version, numel(carpetas));

G = 9.81;
V_MIN_SIGNOS = 10;          % m/s, para los signos
MAX_DESFASE_S = 1.0;

TS = table(); TC = table(); TRg = table(); TU = table();
conj.n = {}; conj.a = {}; conj.y = {};
datos = struct('nombre', {}, 'carpeta', {}, 'controlador', {}, 'perfil', {}, 'condicion', {}, 't', {}, 'dcmd', {}, ...
    'dac', {}, 'r', {}, 'rref', {}, 'epsi', {}, 'ey', {}, 'kap', {}, 'xu', {}, 'yu', {}, 'Kus', {}, 'c', {});
for c = 1:numel(carpetas)
    [~, nombre] = fileparts(carpetas{c});
    man = jsondecode(fileread(fullfile(carpetas{c}, 'manifiesto.json')));
    % La condición la escribe correr_trialsteer.py en meta.paper_q2.condicion ("SIL" o "HIL-U");
    % las corridas del paper 1 no la tienen y son SIL.
    condicion = "SIL";
    if isfield(man, 'meta') && isfield(man.meta, 'paper_q2') && isfield(man.meta.paper_q2, 'condicion')
        condicion = string(man.meta.paper_q2.condicion);
    end
    T = readtable(fullfile(carpetas{c}, 'telemetria.csv'), 'TextType', 'string');
    T = T(T.fase_vuelta == "medida", :);
    ctrl = string(man.meta.controlador); perfil = string(man.meta.perfil); sesion = man.meta.sesion;
    rueda = man.config.direccion.rueda_rad_eje_completo;

    t = T.t_s; v = T.v_kmh / 3.6;
    dcmd = T.delta_aplicado_rad; dac = T.steer_angle_ac * rueda;
    r = T.tasa_guinada_local; ay = G * T.acc_g_x; beta = T.deriva_rad;
    epsi = T.e_psi_cg_rad; ey = T.e_y_cg_m; kap = T.kappa_cg;
    L = median(T.L_medida_m(isfinite(T.L_medida_m)));
    if isempty(L) || isnan(L), L = median(T.L_usada_m(isfinite(T.L_usada_m))); end

    % 1. Signos y consistencia
    s = v > V_MIN_SIGNOS & isfinite(r);
    c_rk = corr1(r(s), v(s) .* kap(s));
    c_dr = corr1(dac(s), r(s));
    c_ay = corr1(ay(s), v(s) .* r(s)); p_ay = polyfit(v(s) .* r(s), ay(s), 1);
    c_dd = corr1(dac, dcmd); p_dd = polyfit(dcmd, dac, 1);
    s_rk = sign(c_rk); s_dr = sign(c_dr); s_ay = sign(corr1(ay(s), r(s)));
    TS = [TS; table(string(nombre), ctrl, perfil, sesion, condicion, c_rk, c_dr, c_ay, p_ay(1), c_dd, p_dd(1), L, ...
        'VariableNames', {'corrida', 'controlador', 'perfil', 'sesion', 'condicion', 'corr_r_vkappa', ...
        'corr_dac_r', 'corr_ay_vr', 'pendiente_ay_vr', 'corr_dac_dcmd', 'pendiente_dac_dcmd', 'L_m'})]; %#ok<AGROW>

    % 2. Métricas de la cadena
    rref = s_rk * v .* kap; er = r - rref;
    kabs = abs(kap);
    region = repmat("media", height(T), 1);
    region(kabs < 1 / man.config.regiones.radio_baja_m) = "baja";
    region(kabs > 1 / man.config.regiones.radio_alta_m) = "alta";
    Ts = median(diff(t));
    desf = desfase(s_dr * dac, r, Ts, MAX_DESFASE_S);
    m = metricas(true(height(T), 1), ey, epsi, er, r, ay, beta);
    TC = [TC; [table(string(nombre), ctrl, perfil, sesion, condicion, 1000 * desf, ...
        'VariableNames', {'corrida', 'controlador', 'perfil', 'sesion', 'condicion', 'desfase_dac_r_ms'}), struct2table(m)]]; %#ok<AGROW>
    for rg = ["baja", "media", "alta"]
        mr = metricas(region == rg, ey, epsi, er, r, ay, beta);
        TRg = [TRg; [table(string(nombre), ctrl, perfil, sesion, condicion, rg, ...
            'VariableNames', {'corrida', 'controlador', 'perfil', 'sesion', 'condicion', 'region'}), struct2table(mr)]]; %#ok<AGROW>
    end

    % 3. Subviraje cuasi estacionario
    ays = movmean(ay, 5); rs = movmean(r, 5);
    q = v > 30 / 3.6 & abs(ays) > 0.5 & abs(gradient(ays, t)) < 1.0 & abs(gradient(rs, t)) < 0.05 & isfinite(r);
    xu = s_ay * ay(q); yu = s_dr * dac(q) - L * r(q) ./ v(q);
    p = polyfit(xu, yu, 1);
    r2 = 1 - sum((yu - polyval(p, xu)) .^ 2) / sum((yu - mean(yu)) .^ 2);
    % 4. Conjunta: L·|r|/v = g·n' − K_us·a_y', con n' y a_y' en el sentido del giro
    sg = sign(r(q)); n1 = sg .* s_dr .* T.steer_angle_ac(q); a1 = sg .* s_ay .* ay(q);
    yj = L * abs(r(q)) ./ v(q);
    X = [n1, -a1]; bj = X \ yj;
    r2j = 1 - sum((yj - X * bj) .^ 2) / sum((yj - mean(yj)) .^ 2);
    TU = [TU; table(string(nombre), ctrl, perfil, sesion, condicion, nnz(q), p(1), rad2deg(p(1)) * G, ...
        rad2deg(p(2)), r2, max(abs(ay(q))), bj(1), rad2deg(bj(2)) * G, r2j, cond(X), ...
        'VariableNames', {'corrida', 'controlador', 'perfil', 'sesion', 'condicion', 'n_puntos', ...
        'Kus_rad_por_ms2', 'Kus_deg_por_g', 'intercepto_deg', 'R2', 'ay_max_ms2', ...
        'g_conjunta_rad', 'Kus_conjunto_deg_por_g', 'R2_conjunto', 'cond_X'})]; %#ok<AGROW>
    conj.n{end + 1} = n1; conj.a{end + 1} = a1; conj.y{end + 1} = yj;

    datos(end + 1) = struct('nombre', string(nombre), 'carpeta', carpetas{c}, 'controlador', ctrl, 'perfil', perfil, 'condicion', condicion, ...
        't', t, 'dcmd', dcmd, 'dac', dac, 'r', r, 'rref', rref, 'epsi', epsi, 'ey', ey, 'kap', kap, ...
        'xu', xu, 'yu', yu, 'Kus', p(1), 'c', p(2)); %#ok<SAGROW>
end

writetable(TS, fullfile(dres, 's15_signos.csv'));
writetable(TC, fullfile(dres, 's15_corridas.csv'));
writetable(TRg, fullfile(dres, 's15_regiones.csv'));
writetable(TU, fullfile(dres, 's15_subviraje.csv'));
fprintf('\nSignos y consistencia:\n'); disp(TS(:, 2:end))
fprintf('Cadena por corrida:\n');
disp(TC(:, {'controlador', 'perfil', 'condicion', 'rms_ey_m', 'rms_epsi_deg', 'rms_er_dps', 'rms_r_dps', ...
    'rms_ay_ms2', 'p95_beta_deg', 'desfase_dac_r_ms'}))
fprintf('Subviraje cuasi estacionario:\n'); disp(TU(:, 2:end))
% Conjunta con todas las corridas juntas (más variedad de velocidades)
Xa = [vertcat(conj.n{:}), -vertcat(conj.a{:})]; ya = vertcat(conj.y{:});
ba = Xa \ ya; r2a = 1 - sum((ya - Xa * ba) .^ 2) / sum((ya - mean(ya)) .^ 2);
rng(20261007); nboot = 2000; bb = nan(nboot, 2); nc = numel(conj.n);
for b = 1:nboot
    idx = randi(nc, nc, 1);
    Xb = [vertcat(conj.n{idx}), -vertcat(conj.a{idx})]; yb = vertcat(conj.y{idx});
    bb(b, :) = (Xb \ yb)';
end
ic_g = prctile(bb(:, 1), [2.5 97.5]); ic_k = prctile(rad2deg(bb(:, 2)) * G, [2.5 97.5]);
fprintf(['Estimación conjunta con %d corridas (%d puntos): g = %.4f rad (IC95 %.4f–%.4f; configurada 0,403), ' ...
    'K_us = %.2f °/g (IC95 %.2f–%.2f), R² = %.3f, cond(X) = %.1f\n'], nc, numel(ya), ba(1), ic_g(1), ic_g(2), ...
    rad2deg(ba(2)) * G, ic_k(1), ic_k(2), r2a, cond(Xa));
TJ = table(nc, numel(ya), ba(1), ic_g(1), ic_g(2), rad2deg(ba(2)) * G, ic_k(1), ic_k(2), r2a, cond(Xa), ...
    'VariableNames', {'n_corridas', 'n_puntos', 'g_rad', 'g_ic95_inf', 'g_ic95_sup', 'Kus_deg_por_g', ...
    'Kus_ic95_inf', 'Kus_ic95_sup', 'R2', 'cond_X'});
writetable(TJ, fullfile(dres, 's15_ganancia_y_subviraje.csv'));
fprintf('Región alta curvatura:\n');
disp(TRg(TRg.region == "alta", {'controlador', 'perfil', 'rms_ey_m', 'rms_epsi_deg', 'rms_er_dps', 'p95_beta_deg'}))

%% Figuras
orden = ["pure_pursuit", "stanley", "mpc_cinematico"];
nombres = ["Pure Pursuit", "Stanley", "MPC cinemático"];
col = lines(3);
figs = gobjects(0);

% F1. La cadena en la curva más cerrada, perfil nominal, condición SIL
f1 = figure('Name', 'F1 cadena', 'Position', [30 30 1400 850]);
tl = tiledlayout(4, 3, 'TileSpacing', 'compact');
for j = 1:3
    i = find([datos.controlador] == orden(j) & [datos.perfil] == "nominal" & [datos.condicion] == "SIL", 1);
    if isempty(i), continue; end
    D = datos(i);
    [~, kmax] = max(abs(D.kap));
    w = D.t >= D.t(kmax) - 6 & D.t <= D.t(kmax) + 6;
    tt = D.t(w) - D.t(kmax);
    nexttile(j); plot(tt, rad2deg(D.dcmd(w)), 'k:', tt, rad2deg(D.dac(w)), 'Color', col(j, :)); grid on
    title(nombres(j)); if j == 1, ylabel('\delta rueda (°)'); legend('\delta_{cmd}', '\delta_{AC}', 'Location', 'best'); end
    nexttile(j + 3); plot(tt, rad2deg(D.rref(w)), 'k:', tt, rad2deg(D.r(w)), 'Color', col(j, :)); grid on
    if j == 1, ylabel('r (°/s)'); legend('r_{ref} = s·v·\kappa', 'r', 'Location', 'best'); end
    nexttile(j + 6); plot(tt, rad2deg(D.epsi(w)), 'Color', col(j, :)); grid on
    if j == 1, ylabel('e_\psi (°)'); end
    nexttile(j + 9); plot(tt, D.ey(w), 'Color', col(j, :)); grid on
    if j == 1, ylabel('e_y (m)'); end
    xlabel('t desde la curvatura máxima (s)');
end
title(tl, 'Cadena δ → r → e_\psi → e_y en la curva más cerrada (perfil nominal, SIL del paper 1)');
figs(end + 1) = f1;

% F2. Subviraje
f2 = figure('Name', 'F2 subviraje', 'Position', [50 50 900 600]); hold on; grid on
h = gobjects(1, 3);
bordes = 0.5:0.5:12;
for i = 1:numel(datos)
    j = find(orden == datos(i).controlador);
    xb = nan(numel(bordes) - 1, 1); yb = xb;
    for b = 1:numel(bordes) - 1
        k = datos(i).xu >= bordes(b) & datos(i).xu < bordes(b + 1);
        if nnz(k) >= 5, xb(b) = median(datos(i).xu(k)); yb(b) = median(rad2deg(datos(i).yu(k))); end
    end
    h(j) = plot(xb, yb, 'o-', 'Color', col(j, :), 'MarkerSize', 4);
end
xmax = max(cellfun(@max, {datos.xu})); xx = [0.5 xmax];
Kmed = median([datos.Kus]); cmed = median([datos.c]);
plot(xx, rad2deg(Kmed * xx + cmed), 'k--', 'LineWidth', 1.2);
xlabel('a_y (m/s^2, con el signo de la guiñada)'); ylabel('\delta_{AC} − L·r/v (°)');
legend([h(isgraphics(h)) findobj(gca, 'LineStyle', '--')], [nombres(isgraphics(h)) ...
    sprintf('ajuste mediano K_{us} = %.2f °/g', rad2deg(Kmed) * G)], 'Location', 'northwest');
xlim([0 ceil(xmax) + 0.5]);
title({'Subviraje cuasi estacionario (medianas por franja de 0,5 m/s^2, un trazo por corrida)', ...
    sprintf('R^2 de los ajustes por corrida: %.2f–%.2f', min(TU.R2), max(TU.R2))});
figs(end + 1) = f2;

% F3. Signos, primera corrida nominal de Stanley
i = find([datos.controlador] == "stanley" & [datos.perfil] == "nominal", 1);
if isempty(i), i = 1; end
D = datos(i);
f3 = figure('Name', 'F3 signos', 'Position', [50 50 1300 400]);
tiledlayout(1, 3, 'TileSpacing', 'compact');
T0 = readtable(fullfile(D.carpeta, 'telemetria.csv'), 'TextType', 'string');
T0 = T0(T0.fase_vuelta == "medida", :); v0 = T0.v_kmh / 3.6;
nexttile; plot(rad2deg(v0 .* T0.kappa_cg), rad2deg(T0.tasa_guinada_local), '.', 'MarkerSize', 3); grid on
xlabel('v·\kappa (°/s)'); ylabel('r (°/s)'); title('Guiñada frente a la pedida por la trazada');
nexttile; plot(v0 .* T0.tasa_guinada_local, G * T0.acc_g_x, '.', 'MarkerSize', 3); grid on
xlabel('v·r (m/s^2)'); ylabel('9,81·acc\_g\_x (m/s^2)'); title('Aceleración lateral');
nexttile; plot(rad2deg(D.dcmd), rad2deg(D.dac), '.', 'MarkerSize', 3); grid on
xlabel('\delta_{cmd} (°)'); ylabel('\delta_{AC} (°)'); title('Ángulo aplicado por AC');
figs(end + 1) = f3;

% F4. RMS por controlador y perfil (SIL)
f4 = figure('Name', 'F4 RMS', 'Position', [50 50 1300 420]);
tiledlayout(1, 3, 'TileSpacing', 'compact');
perfiles = ["conservador", "nominal", "rapido"];
vars = {'rms_ey_m', 'rms_epsi_deg', 'rms_er_dps'}; etiquetas = {'RMS e_y (m)', 'RMS e_\psi (°)', 'RMS e_r (°/s)'};
for q = 1:3
    nexttile; M = nan(3, 3);
    for a = 1:3
        for p = 1:3
            s = TC.controlador == orden(a) & TC.perfil == perfiles(p) & TC.condicion == "SIL";
            if any(s), M(a, p) = median(TC.(vars{q})(s)); end
        end
    end
    bar(categorical(nombres, nombres), M); grid on; ylabel(etiquetas{q});
    if q == 1, legend(perfiles, 'Location', 'northwest'); end
end
figs(end + 1) = f4;

for j = 1:numel(figs)
    exportgraphics(figs(j), fullfile(dfig, sprintf('F%d.png', j)), 'Resolution', 200);
    savefig(figs(j), fullfile(dfig, sprintf('F%d.fig', j)));
end
fprintf('Resultados en %s\nFiguras en %s\n', dres, dfig);
diary('off');

function c = corr1(x, y)
    k = isfinite(x) & isfinite(y);
    R = corrcoef(x(k), y(k)); c = R(1, 2);
end

function m = metricas(sel, ey, epsi, er, r, ay, beta)
    m.n = nnz(sel);
    m.rms_ey_m = sqrt(mean(ey(sel) .^ 2, 'omitnan'));
    m.rms_epsi_deg = rad2deg(sqrt(mean(epsi(sel) .^ 2, 'omitnan')));
    m.rms_er_dps = rad2deg(sqrt(mean(er(sel) .^ 2, 'omitnan')));
    m.rms_r_dps = rad2deg(sqrt(mean(r(sel) .^ 2, 'omitnan')));
    m.rms_ay_ms2 = sqrt(mean(ay(sel) .^ 2, 'omitnan'));
    m.max_ay_ms2 = max(abs(ay(sel)));
    m.p95_beta_deg = rad2deg(prctile(abs(beta(sel)), 95));
end

function tau = desfase(x, y, Ts, maxs)
    % Desfase (s) de y respecto de x que maximiza la correlación, entre 0 y maxs.
    x = x - mean(x, 'omitnan'); y = y - mean(y, 'omitnan');
    x(~isfinite(x)) = 0; y(~isfinite(y)) = 0;
    lags = 0:round(maxs / Ts);
    c = arrayfun(@(l) sum(x(1:end - l) .* y(1 + l:end)) / sqrt(sum(x(1:end - l) .^ 2) * sum(y(1 + l:end) .^ 2)), lags);
    [~, k] = max(c); tau = lags(k) * Ts;
end
