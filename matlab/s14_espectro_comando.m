%% s14_espectro_comando.m
% Exigencia del comando de dirección de cada controlador: insumo ex ante de los índices
% de compatibilidad Γ_E y Λ (Q2-D-020) y de la discusión de k (Q2-P-021).
% Datos: corridas SIL de TrialSteer v1.0.1 (telemetria.csv + manifiesto.json):
%   data/raw/p00_referencia/campana_*      campaña del paper 1 (SIL)
%   data/raw/trialsteer/SIL_*              corridas SIL nuevas, si existen
% Solo SIL a propósito: el índice se calcula antes de HIL, así no hay fuga de información
% (ANALYSIS.md, Threats to validity, punto 3). Para otra lista, definir antes 'carpetas'.
% Inferencia: DESCRIPTIVA. Estado: EXPLORATORIO. Solo MATLAB base (sin Signal Processing).
%
% Señal: δ_cmd = delta_aplicado_rad, lo que TrialSteer manda a vJoy en SIL y a SalidaODrive
% en HIL. Solo la vuelta medida (fase_vuelta = "medida"). Se pasa a grados de volante con
% θ_cmd = k·δ_cmd (k en grados de volante por grado de rueda).
%
% Por corrida:
%   Espectro (rejilla uniforme de 20 Hz con retén de orden cero; Welch con Hann, 50 %,
%   segmentos de 25,6 s → Δf ≈ 0,04 Hz; f_Nyq = 10 Hz; se resta la media):
%     f50, f90, f95 y f99 de S_θ (Λ usa f95, Q2-D-020) y de la tasa, S_θ̇ = (2πf)²·S_θ
%     fracción de la energía de θ y de θ̇ por encima de 0,5, 1, 2 y 5 Hz
%   Tiempo (ciclos reales, dependen de k): RMS de θ; RMS, p99 y máximo de |θ̇|; fracción de
%     ciclos con |θ̇| > 720 °/s (vel_limit) y > 864 °/s (corte de SalidaODrive); mediana de
%     |Δθ| entre los ciclos en que el comando cambia y fracción por debajo de 0,5°, 1° y 2°
%     (la zona de fricción medida con c01); inversiones de sentido por segundo.
%   Lo mismo en el tiempo por región de curvatura (|κ| < 1/500, media, |κ| > 1/100 m⁻¹),
%   con los radios del paper 1.
% Las métricas de tiempo se calculan con k = 7,8 (360°), k ≈ 22,0 (1014° = 2 × STEER_LOCK de AC, Q2-D-029,
% alternativa) y k ≈ 19,5 (900° = LOCK del volante vJoy en AC, Q2-D-035, el vigente). vel_limit = 720 °/s.
%
% Salidas
%   results/exploratory/espectro_comando/s14_corridas.csv   una fila por corrida y k
%   results/exploratory/espectro_comando/s14_regiones.csv   una fila por corrida, k y región
%   results/exploratory/espectro_comando/s14_psd.csv        f, S_θ por corrida (k ≈ 19,5, 900°)
%   figures/draft/espectro_comando/F1–F4 (.png y .fig)
%   logs/s14_espectro_comando.log
% Uso:  matlab -batch "run('scripts/s14_espectro_comando.m')"
%
% Guía de observación para el investigador (AGENTS.md §17)
%   F1  Espectro de θ_cmd y de su tasa por controlador (perfil nominal). La energía de θ está
%       casi toda por debajo de 0,5 Hz: es seguir las curvas de Monza. La diferencia entre
%       controladores aparece en la cola de alta frecuencia y se ve mejor en el espectro de la tasa.
%   F2  Energía acumulada. ¿Distingue f95 de θ entre PP, Stanley y MPC, o hace falta mirar la
%       tasa? Si f95 de θ es casi igual en los tres, Λ con esa definición no separará
%       controladores y conviene definirlo sobre la tasa (decisión antes de la campaña).
%   F3  Tamaño de los cambios por ciclo. La franja sombreada es la zona donde el volante se pega
%       (c01). Compare k = 7,8, 22,0 y 19,5: ¿saca el rango vigente (900°) los cambios de la zona de fricción,
%       como supone Q2-D-019?
%   F4  f95 por controlador y perfil. ¿Sube la exigencia con la velocidad del perfil?
%   Preguntas: ¿el orden de exigencia es PP < Stanley < MPC en todas las métricas? ¿Cuántos
%   ciclos pasan de vel_limit con k ≈ 19,5 y en qué región ocurren?

if ~exist('raiz', 'var')
    raiz = fileparts(fileparts(mfilename('fullpath')));
end
if ~exist('carpetas', 'var')
    d = [dir(fullfile(raiz, 'data', 'raw', 'p00_referencia', 'campana_*')); ...
         dir(fullfile(raiz, 'data', 'raw', 'trialsteer', 'SIL_*'))];
    d = d([d.isdir]);
    carpetas = fullfile({d.folder}, {d.name});
end
assert(~isempty(carpetas), 'No hay corridas SIL');

dres = fullfile(raiz, 'results', 'exploratory', 'espectro_comando');
dfig = fullfile(raiz, 'figures', 'draft', 'espectro_comando');
dlog = fullfile(raiz, 'logs');
for dd = {dres, dfig, dlog}
    if ~exist(dd{1}, 'dir'), mkdir(dd{1}); end
end
flog = fullfile(dlog, 's14_espectro_comando.log');
if isfile(flog), delete(flog); end
diary(flog);
fprintf('s14_espectro_comando.m  %s  MATLAB %s\n%d corridas\n', ...
    char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), version, numel(carpetas));

RUEDA_EJE = 0.403;                         % rad de rueda con el eje de vJoy completo (paper 1)
KS = [180, 507, 450] / rad2deg(RUEDA_EJE);   % k: 360°, 1014° (Q2-D-029) y 900° (Q2-D-035, vigente = KS(3))
FS = 20;                                   % Hz, periodo del controlador
LSEG = 512;                                % muestras por segmento de Welch (25,6 s)
F_CORTE = [0.5 1 2 5];                     % Hz
VEL_LIM = 720; CORTE_OMEGA = 1.2 * 720;    % °/s: vel_limit de 2 vueltas/s y corte de SalidaODrive
UMBRALES_DTH = [0.5 1 2];                  % ° de volante (zona de fricción, c01)
CUANTILES = [0.5 0.9 0.95 0.99];

TC = table(); TRg = table(); PSD = table();
curvas = struct('controlador', {}, 'perfil', {}, 'sesion', {}, 'f', {}, 'S', {}, 'cum', {}, 'cumr', {}, ...
    'dth', {});
for c = 1:numel(carpetas)
    [~, nombre] = fileparts(carpetas{c});
    man = jsondecode(fileread(fullfile(carpetas{c}, 'manifiesto.json')));
    % La condición la escribe correr_trialsteer.py en meta.paper_q2.condicion ("SIL" o "HIL-U");
    % las corridas del paper 1 no la tienen y son SIL.
    condicion = "SIL";
    if isfield(man, 'meta') && isfield(man.meta, 'paper_q2') && isfield(man.meta.paper_q2, 'condicion')
        condicion = string(man.meta.paper_q2.condicion);
    end
    if condicion ~= "SIL"
        fprintf('Se salta %s: condición %s (este análisis es solo SIL)\n', nombre, condicion);
        continue
    end
    T = readtable(fullfile(carpetas{c}, 'telemetria.csv'), 'TextType', 'string');
    T = T(T.fase_vuelta == "medida", :);
    if height(T) < 500
        fprintf('Se salta %s: no tiene vuelta medida completa (%d muestras)\n', nombre, height(T));
        continue
    end
    ctrl = string(man.meta.controlador); perfil = string(man.meta.perfil); sesion = man.meta.sesion;
    t = T.t_s; delta_deg = rad2deg(T.delta_aplicado_rad);
    kap = abs(T.kappa_cg);
    region = repmat("media", height(T), 1);
    r_baja = 500.0; r_alta = 100.0;
    if isfield(man.config, 'regiones')
        r_baja = man.config.regiones.radio_baja_m;
        r_alta = man.config.regiones.radio_alta_m;
    end
    region(kap < 1 / r_baja) = "baja";
    region(kap > 1 / r_alta) = "alta";

    % Espectro (independiente de k en la forma; se guarda con k ≈ 19,5)
    tg = (t(1):1 / FS:t(end))';
    th = KS(3) * interp1(t, delta_deg, tg, 'previous');
    [f, S] = psd_welch(th - mean(th), FS, LSEG);
    Sr = (2 * pi * f) .^ 2 .* S;
    cum = cumsum(S) / sum(S); cumr = cumsum(Sr) / sum(Sr);
    fq = arrayfun(@(q) f(find(cum >= q, 1)), CUANTILES);
    fqr = arrayfun(@(q) f(find(cumr >= q, 1)), CUANTILES);
    sobre = arrayfun(@(fc) 1 - cum(find(f < fc, 1, 'last')), F_CORTE);
    sobrer = arrayfun(@(fc) 1 - cumr(find(f < fc, 1, 'last')), F_CORTE);
    PSD = [PSD; table(repmat(string(nombre), numel(f), 1), repmat(ctrl, numel(f), 1), repmat(perfil, numel(f), 1), ...
        repmat(sesion, numel(f), 1), f, S, 'VariableNames', {'corrida', 'controlador', 'perfil', 'sesion', 'f_hz', 'S_deg2_hz'})]; %#ok<AGROW>

    dur = t(end) - t(1);
    fila_esp = table(f(2) - f(1), fq(1), fq(2), fq(3), fq(4), fqr(1), fqr(2), fqr(3), fqr(4), ...
        sobre(1), sobre(2), sobre(3), sobre(4), sobrer(1), sobrer(2), sobrer(3), sobrer(4), ...
        'VariableNames', {'df_hz', 'f50_th_hz', 'f90_th_hz', 'f95_th_hz', 'f99_th_hz', ...
        'f50_tasa_hz', 'f90_tasa_hz', 'f95_tasa_hz', 'f99_tasa_hz', ...
        'E_th_sobre_0p5hz', 'E_th_sobre_1hz', 'E_th_sobre_2hz', 'E_th_sobre_5hz', ...
        'E_tasa_sobre_0p5hz', 'E_tasa_sobre_1hz', 'E_tasa_sobre_2hz', 'E_tasa_sobre_5hz'});

    dth = cell(1, numel(KS));
    for j = 1:numel(KS)
        thc = KS(j) * delta_deg;
        m = metricas_tiempo(t, thc, true(height(T), 1), VEL_LIM, CORTE_OMEGA, UMBRALES_DTH);
        dth{j} = abs(diff(thc)); dth{j} = dth{j}(dth{j} > 1e-9);
        TC = [TC; [table(string(nombre), ctrl, perfil, sesion, condicion, KS(j), dur, ...
            'VariableNames', {'corrida', 'controlador', 'perfil', 'sesion', 'condicion', 'k', 'duracion_vuelta_s'}), ...
            struct2table(m), fila_esp]]; %#ok<AGROW>
        for rg = ["baja", "media", "alta"]
            mr = metricas_tiempo(t, thc, region == rg, VEL_LIM, CORTE_OMEGA, UMBRALES_DTH);
            TRg = [TRg; [table(string(nombre), ctrl, perfil, sesion, KS(j), rg, ...
                'VariableNames', {'corrida', 'controlador', 'perfil', 'sesion', 'k', 'region'}), struct2table(mr)]]; %#ok<AGROW>
        end
    end
    curvas(end + 1) = struct('controlador', ctrl, 'perfil', perfil, 'sesion', sesion, 'f', f, 'S', S, ...
        'cum', cum, 'cumr', cumr, 'dth', {dth}); %#ok<SAGROW>
end

writetable(TC, fullfile(dres, 's14_corridas.csv'));
writetable(TRg, fullfile(dres, 's14_regiones.csv'));
writetable(PSD, fullfile(dres, 's14_psd.csv'));
k19 = abs(TC.k - KS(3)) < 1e-9;
disp(TC(k19, {'controlador', 'perfil', 'sesion', 'f95_th_hz', 'f95_tasa_hz', 'E_tasa_sobre_1hz', ...
    'E_tasa_sobre_2hz', 'rms_tasa_dps', 'p99_tasa_dps', 'frac_sobre_vel_limit', 'mediana_dth_deg', ...
    'frac_dth_bajo_1deg', 'inversiones_por_s'}))
k7 = abs(TC.k - KS(1)) < 1e-9;
fprintf('Con k = 7,8 (360°):\n');
disp(TC(k7, {'controlador', 'perfil', 'frac_sobre_vel_limit', 'mediana_dth_deg', 'frac_dth_bajo_0p5deg', ...
    'frac_dth_bajo_1deg', 'frac_dth_bajo_2deg'}))
k12 = abs(TC.k - KS(2)) < 1e-9;
fprintf('Con rango 1014° (k ≈ 22,0, Q2-D-029, alternativa):\n');
disp(TC(k12, {'controlador', 'perfil', 'p99_tasa_dps', 'max_tasa_dps', 'frac_sobre_vel_limit', 'mediana_dth_deg', ...
    'frac_dth_bajo_0p5deg', 'frac_dth_bajo_1deg', 'frac_dth_bajo_2deg'}))
fprintf('Con rango 900° (k ≈ 19,5, Q2-D-035), por región:\n');
r19 = abs(TRg.k - KS(3)) < 1e-9;
disp(TRg(r19, {'controlador', 'perfil', 'region', 'duracion_s', 'rms_tasa_dps', 'frac_sobre_vel_limit', ...
    'frac_dth_bajo_1deg', 'inversiones_por_s'}))

%% Figuras
orden = ["pure_pursuit", "stanley", "mpc_cinematico"];
nombres = ["Pure Pursuit", "Stanley", "MPC cinemático"];
col = lines(3);
figs = gobjects(0);
nom = [curvas.perfil] == "nominal";

% F1. Espectros, perfil nominal
f1 = figure('Name', 'F1 espectros', 'Position', [50 50 1100 700]);
tl = tiledlayout(2, 1, 'TileSpacing', 'compact');
ax1 = nexttile; ax2 = nexttile;
for a = [ax1 ax2], hold(a, 'on'); grid(a, 'on'); set(a, 'XScale', 'log', 'YScale', 'log'); end
h = gobjects(1, 3);
for i = find(nom)
    j = find(orden == curvas(i).controlador);
    h(j) = plot(ax1, curvas(i).f(2:end), curvas(i).S(2:end), 'Color', col(j, :));
    plot(ax2, curvas(i).f(2:end), (2 * pi * curvas(i).f(2:end)) .^ 2 .* curvas(i).S(2:end), 'Color', col(j, :));
end
ylabel(ax1, 'S_\theta (°^2/Hz)'); ylabel(ax2, 'S_{d\theta/dt} ((°/s)^2/Hz)'); xlabel(ax2, 'f (Hz)');
legend(ax1, h(isgraphics(h)), nombres(isgraphics(h)), 'Location', 'southwest');
title(tl, 'Espectro del comando en grados de volante (k ≈ 19,5, rango 900°), perfil nominal, SIL del paper 1');
figs(end + 1) = f1;

% F2. Energía acumulada
f2 = figure('Name', 'F2 energía acumulada', 'Position', [50 50 1100 450]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact');
estilos = struct('conservador', ':', 'nominal', '-', 'rapido', '--');
for q = 1:2
    nexttile; hold on; grid on; set(gca, 'XScale', 'log');
    for i = 1:numel(curvas)
        j = find(orden == curvas(i).controlador);
        if q == 1, y = curvas(i).cum; else, y = curvas(i).cumr; end
        plot(curvas(i).f(2:end), y(2:end), estilos.(char(curvas(i).perfil)), 'Color', col(j, :));
    end
    yline(0.95, 'k:', '95 %'); ylim([0 1]); xlabel('f (Hz)'); ylabel('fracción acumulada de la energía');
    if q == 1, title('\theta_{cmd}'); else, title('d\theta_{cmd}/dt'); end
end
title(tl, 'Color = controlador (azul PP, rojo Stanley, amarillo MPC); línea = perfil (: conservador, — nominal, -- rápido)');
figs(end + 1) = f2;

% F3. Tamaño de los cambios por ciclo, perfil nominal
f3 = figure('Name', 'F3 cambios por ciclo', 'Position', [50 50 1500 450]);
tl = tiledlayout(1, numel(KS), 'TileSpacing', 'compact');
for q = 1:numel(KS)
    nexttile; hold on; grid on; set(gca, 'XScale', 'log');
    patch([0.01 1 1 0.01], [0 0 1 1], [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    h = gobjects(1, 3);
    for i = find(nom)
        j = find(orden == curvas(i).controlador);
        x = sort(curvas(i).dth{q});
        h(j) = plot(x, (1:numel(x))' / numel(x), 'Color', col(j, :), 'LineWidth', 1.2);
    end
    xlim([0.01 30]); xlabel('|\Delta\theta_{cmd}| por ciclo (° de volante)'); ylabel('fracción acumulada');
    title(sprintf('k = %.1f (%.0f° de rango)', KS(q), 2 * KS(q) * rad2deg(RUEDA_EJE)));
    legend(h(isgraphics(h)), nombres(isgraphics(h)), 'Location', 'southeast');
end
title(tl, 'Cambios del comando por ciclo de 50 ms (sombreado: < 1°, zona donde el volante se pega según c01)');
figs(end + 1) = f3;

% F4. f95 por controlador y perfil
f4 = figure('Name', 'F4 f95', 'Position', [50 50 1000 420]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact');
perfiles = ["conservador", "nominal", "rapido"];
for q = 1:2
    nexttile; hold on; grid on
    M = nan(3, 3);
    for a = 1:3
        for p = 1:3
            s = k19 & TC.controlador == orden(a) & TC.perfil == perfiles(p);
            if q == 1, v = TC.f95_th_hz(s); else, v = TC.f95_tasa_hz(s); end
            if ~isempty(v), M(a, p) = median(v); end
        end
    end
    bar(categorical(nombres, nombres), M);
    ylabel('f95 (Hz)'); legend(perfiles, 'Location', 'northwest');
    if q == 1, title('f95 de \theta_{cmd} (definición de \Lambda)'); else, title('f95 de d\theta_{cmd}/dt'); end
end
title(tl, 'Frecuencia bajo la que está el 95 % de la energía (mediana entre sesiones)');
figs(end + 1) = f4;

for j = 1:numel(figs)
    exportgraphics(figs(j), fullfile(dfig, sprintf('F%d.png', j)), 'Resolution', 200);
    savefig(figs(j), fullfile(dfig, sprintf('F%d.fig', j)));
end
fprintf('Resultados en %s\nFiguras en %s\n', dres, dfig);
diary('off');

function m = metricas_tiempo(t, th, sel, vel_lim, corte, umbrales)
    % Métricas en ciclos reales: el incremento k se atribuye al ciclo k si este pertenece a sel.
    dth = diff(th); dt = diff(t); tasa = dth ./ dt;
    s = sel(2:end) & dt > 0;
    d = dth(s); w = tasa(s);
    cambia = abs(d) > 1e-9;
    dc = d(cambia);
    signos = sign(dc);
    m.duracion_s = sum(dt(s));
    m.rms_th_deg = sqrt(mean(th(sel) .^ 2));
    m.rms_tasa_dps = sqrt(mean(w .^ 2));
    m.p99_tasa_dps = prctile(abs(w), 99);
    m.max_tasa_dps = max(abs(w));
    m.frac_sobre_vel_limit = mean(abs(w) > vel_lim);
    m.frac_sobre_corte = mean(abs(w) > corte);
    m.mediana_dth_deg = median(abs(dc));
    m.frac_dth_bajo_0p5deg = mean(abs(dc) < umbrales(1));
    m.frac_dth_bajo_1deg = mean(abs(dc) < umbrales(2));
    m.frac_dth_bajo_2deg = mean(abs(dc) < umbrales(3));
    m.inversiones_por_s = sum(signos(2:end) ~= signos(1:end - 1)) / m.duracion_s;
end

function [f, S] = psd_welch(x, fs, L)
    % Densidad espectral de una cara (unidades²/Hz), Hann, 50 % de solapamiento.
    x = x(:); N = numel(x); L = min(L, N); paso = floor(L / 2);
    w = 0.5 - 0.5 * cos(2 * pi * (0:L - 1)' / (L - 1)); U = sum(w .^ 2);
    acc = 0; n = 0;
    for i0 = 1:paso:(N - L + 1)
        X = fft(w .* x(i0:i0 + L - 1));
        acc = acc + abs(X) .^ 2; n = n + 1;
    end
    nf = floor(L / 2) + 1;
    S = acc(1:nf) / (n * fs * U);
    S(2:end - 1) = 2 * S(2:end - 1);
    f = (0:nf - 1)' * fs / L;
end
