%% s13_identificacion_banco.m
% Identificación del actuador de dirección (ítem A de Q2-D-018) con los datos de
% scripts/acq/c03_caracterizacion_formal.py.
% Datos: data/raw/caracterizacion_formal/<fecha>_formal*/ (muestras.csv, bloques.csv,
%        escalones.csv, manifiesto.json). Ángulos en grados de volante.
% Inferencia: DESCRIPTIVA. Estado: EXPLORATORIO hasta congelar el protocolo.
% Toolboxes: System Identification (iddata, procest, compare, aic) y Control System
% (freqresp). No usa Signal Processing ni Statistics: la FRF empírica y la coherencia
% se calculan con FFT (Welch con ventana de Hann escrito aquí).
%
% Qué calcula
%   Bloques: fracción de muestras con |ω| ≥ 0,95·vel_limit o |Iq| ≥ 0,95·current_lim.
%     Un bloque saturado no describe un sistema lineal y se marca.
%   Escalones: t_mov, t50, t90, sobrepaso, error final y velocidad pico, por amplitud
%     y por sentido (asimetría izquierda/derecha).
%   Rampas: ancho del lazo de histéresis (cmd − meas subiendo frente a bajando),
%     volante quieto con la referencia en movimiento y |Iq| con el volante pegado.
%   Tasa: velocidad máxima sostenida frente a vel_limit.
%   Chirp y PRBS: FRF empírica (PRBS: H1 de Welch; chirp: registro completo promediado
%     en bandas logarítmicas) y coherencia, después de remuestrear a
%     paso uniforme (consigna con retén de orden cero, medida con interpolación lineal).
%     El muestreo del banco no es uniforme (lectura serial), y iddata lo exige.
%   Modelos por amplitud de chirp (procest):
%     G1 = K·e^(−Td·s)/(Tp1·s + 1)                       'P1D'
%     G2 = K·e^(−Td·s)/(Tw²·s² + 2ζ·Tw·s + 1)            'P2DU'
%     Ajuste con el chirp y validación con el PRBS de amplitud más cercana (una
%     excitación distinta). Se reporta ajuste NRMSE (%), AIC, ancho de banda −3 dB
%     respecto de la ganancia en continua y fase a 1, 3 y 10 Hz. Se elige por el
%     ajuste en validación; sin PRBS, por AIC.
%
% Salidas
%   results/exploratory/caracterizacion_formal/<corrida>_bloques.csv, _escalones.csv,
%     _escalones_resumen.csv, _rampas.csv, _modelos.csv, _frf.csv y _modelos.mat
%     (el .mat guarda los modelos para Γ_E y para el MPC del ítem E)
%   figures/draft/caracterizacion_formal/<corrida>/F1–F7 (.png y .fig)
%   logs/s13_<corrida>.log
%
% Uso:  matlab -batch "run('scripts/s13_identificacion_banco.m')"
%       (toma la corrida más reciente; para otra, definir antes la variable 'carpeta')
%
% Guía de observación para el investigador (AGENTS.md §17)
%   F1  Vista general. ¿Siguió el volante la consigna en todos los bloques? ¿Hubo cortes?
%   F2  Escalones normalizados. ¿Se parecen las curvas de 5°, 20° y 45°? Si la grande es
%       más lenta en proporción, manda el límite de tasa. ¿Izquierda y derecha coinciden?
%   F3  FRF empírica por amplitud. Si la ganancia de baja frecuencia cae en las amplitudes
%       pequeñas (1°, 3°), la fricción actúa como zona muerta y el actuador no es lineal.
%       La coherencia baja indica ruido o no linealidad en esa banda.
%   F4  FRF frente a G1 y G2. ¿Hace falta el 2.º orden (resonancia, fase que cae más rápido)
%       o basta el 1.º? Mire la fase en la banda de 1–10 Hz, que es la del comando.
%   F5  Histéresis en las rampas. El ancho del lazo es el error que la fricción deja con
%       comandos lentos. Compare las dos velocidades.
%   F6  Tasa. ¿La velocidad se aplana en vel_limit? Ese aplanamiento es un límite del
%       actuador que el índice Γ_E no ve si solo usa la FRF lineal.
%   F7  Validación en el PRBS. ¿El modelo elegido reproduce la medida con una excitación
%       que no se usó para ajustarlo?
%   Preguntas para el investigador: ¿la dependencia con la amplitud es la que se espera de
%   la fricción medida en c01 (≈ 0,31–0,36 A de despegue)? ¿Alguna banda está saturada
%   (columna frac_sat_* de _bloques.csv)? ¿Se repiten los parámetros entre sesiones?

if ~exist('raiz', 'var')
    raiz = fileparts(fileparts(mfilename('fullpath')));
end
if ~exist('carpeta', 'var')
    d = dir(fullfile(raiz, 'data', 'raw', 'caracterizacion_formal', '*_formal*'));
    d = d([d.isdir]);
    assert(~isempty(d), ['No hay corridas en data/raw/caracterizacion_formal. ' ...
        'Corre antes scripts/acq/c03_caracterizacion_formal.py']);
    [~, k] = max(datenum({d.date})); %#ok<DATNM>
    carpeta = fullfile(d(k).folder, d(k).name);
end
if ~isfolder(carpeta), carpeta = fullfile(raiz, carpeta); end   % ruta relativa a la raíz del paper
[~, nombre] = fileparts(carpeta);

dres = fullfile(raiz, 'results', 'exploratory', 'caracterizacion_formal');
dfig = fullfile(raiz, 'figures', 'draft', 'caracterizacion_formal', nombre);
dlog = fullfile(raiz, 'logs');
for dd = {dres, dfig, dlog}
    if ~exist(dd{1}, 'dir'), mkdir(dd{1}); end
end
flog = fullfile(dlog, ['s13_' nombre '.log']);
if isfile(flog), delete(flog); end
diary(flog);
fprintf('s13_identificacion_banco.m  %s  MATLAB %s\nCorrida: %s\n', ...
    char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), version, nombre);
assert(~isempty(which('procest')), 'Falta System Identification Toolbox (procest)');

CUENTA = 360 / 2880;      % resolución del encoder en grados de volante
SEG_FRF = 8;              % duración de cada segmento de Welch (s)
COH_MIN = 0.8;            % coherencia mínima para marcar un punto de la FRF como fiable
F_PRBS_MAX = 0.44 / 0.05; % banda útil del PRBS con bits de 50 ms (Hz)

M = readtable(fullfile(carpeta, 'muestras.csv'), 'TextType', 'string');
B = readtable(fullfile(carpeta, 'bloques.csv'), 'TextType', 'string');
man = jsondecode(fileread(fullfile(carpeta, 'manifiesto.json')));
falso = isfield(man.placa, 'FALSO');
if falso, warning('DATOS FALSOS DE PRUEBA (planta simulada): no son evidencia del paper'); end
vel_lim = man.vel_limit_dps;
i_lim = campo(man.placa, 'axis0_motor_config_current_lim', NaN);
fprintf('%d muestras, %d bloques, dt mediano %.2f ms, vel_limit %.0f °/s, current_lim %.1f A, corte: %s\n', ...
    height(M), height(B), 1000 * median(diff(M.t_s)), vel_lim, i_lim, texto(man.corte));

%% Saturación por bloque
nb = height(B);
B.n_muestras = zeros(nb, 1); B.frac_sat_vel = nan(nb, 1); B.frac_sat_iq = nan(nb, 1);
for i = 1:nb
    s = M.bloque == B.bloque(i);
    B.n_muestras(i) = nnz(s);
    B.frac_sat_vel(i) = mean(abs(M.omega_dps(s)) >= 0.95 * vel_lim);
    iq = M.iq_a(s);
    if isfinite(i_lim) && any(isfinite(iq))
        B.frac_sat_iq(i) = mean(abs(iq(isfinite(iq))) >= 0.95 * i_lim);
    end
end
disp(B(:, {'bloque', 'tipo', 'amplitud_deg', 'f_max_hz', 'n_muestras', 'frac_sat_vel', 'frac_sat_iq'}))

%% Escalones
TE = table();
fe = fullfile(carpeta, 'escalones.csv');
if isfile(fe)
    E = readtable(fe);
    sost = campo(man, 'sostener_escalon_s', 1.5);
    n = height(E);
    dirn = sign(E.hacia_deg - E.desde_deg);
    A = abs(E.hacia_deg - E.desde_deg);
    [t_mov, t50, t90, sobre, e_fin, v_pico] = deal(nan(n, 1));
    for i = 1:n
        s = M.t_s >= E.t_cmd_s(i) & M.t_s < E.t_cmd_s(i) + sost;
        t = M.t_s(s) - E.t_cmd_s(i);
        previo = M.t_s < E.t_cmd_s(i) & M.t_s >= E.t_cmd_s(i) - 0.05;
        y = dirn(i) * (M.meas_deg(s) - mean(M.meas_deg(previo)));
        t_mov(i) = primero(t, y >= max(0.2 * A(i), CUENTA));
        t50(i) = primero(t, y >= 0.5 * A(i));
        t90(i) = primero(t, y >= 0.9 * A(i));
        sobre(i) = 100 * max(0, max(y) - A(i)) / A(i);
        e_fin(i) = dirn(i) * (mean(M.meas_deg(s & M.t_s >= E.t_cmd_s(i) + sost - 0.2)) - E.hacia_deg(i));
        v_pico(i) = max(abs(M.omega_dps(s)));
    end
    sentido = repmat("izquierda", n, 1); sentido(dirn > 0) = "derecha";
    tramo = repmat("vuelve_a_0", n, 1); tramo(E.desde_deg == 0) = "sale_de_0";
    TE = table(E.escalon, E.repeticion, E.amplitud_deg, E.desde_deg, E.hacia_deg, sentido, tramo, ...
        1000 * t_mov, 1000 * t50, 1000 * t90, sobre, e_fin, v_pico, ...
        'VariableNames', {'escalon', 'repeticion', 'amplitud_deg', 'desde_deg', 'hacia_deg', 'sentido', ...
        'tramo', 't_mov_ms', 't50_ms', 't90_ms', 'sobrepaso_pct', 'e_fin_deg', 'v_pico_dps'});
    RE = groupsummary(TE, {'amplitud_deg', 'tramo', 'sentido'}, 'median', ...
        {'t_mov_ms', 't50_ms', 't90_ms', 'sobrepaso_pct', 'e_fin_deg', 'v_pico_dps'});
    disp(RE)
    writetable(TE, fullfile(dres, [nombre '_escalones.csv']));
    writetable(RE, fullfile(dres, [nombre '_escalones_resumen.csv']));
end

%% Rampas: histéresis y despegue
TR = table();
for i = find(B.tipo == "rampa")'
    s = M.bloque == B.bloque(i);
    t = M.t_s(s); c = M.cmd_deg(s); m = M.meas_deg(s); err = c - m;
    dc = [0; diff(c)];
    sube = dc > 0; baja = dc < 0;
    ancho = median(err(sube)) - median(err(baja));
    quieto = [];
    for t0 = t(1):0.1:(t(end) - 0.1)
        k = t >= t0 & t < t0 + 0.1;
        if nnz(k) < 2, continue; end
        if abs(c(find(k, 1, 'last')) - c(find(k, 1))) > CUENTA
            quieto(end + 1) = abs(m(find(k, 1, 'last')) - m(find(k, 1))) < CUENTA / 2; %#ok<SAGROW>
        end
    end
    iq = M.iq_a(s); pegado = abs(M.omega_dps(s)) < 2 & abs(err) > CUENTA & isfinite(iq);
    iq_peg = NaN; if any(pegado), iq_peg = prctile(abs(iq(pegado)), 95); end
    TR = [TR; table(B.bloque(i), B.vel_dps(i), ancho, mean(quieto), max(abs(err)), iq_peg, ...
        'VariableNames', {'bloque', 'vel_dps', 'ancho_histeresis_deg', 'frac_quieto_100ms', ...
        'err_max_deg', 'iq_p95_pegado_a'})]; %#ok<AGROW>
end
if ~isempty(TR), disp(TR); writetable(TR, fullfile(dres, [nombre '_rampas.csv'])); end

%% Tasa
v_sost = NaN;
it = find(B.tipo == "tasa", 1);
if ~isempty(it)
    v = abs(M.omega_dps(M.bloque == B.bloque(it)));
    v_sost = median(v(v >= 0.9 * max(v)));
    fprintf('Tasa: velocidad sostenida %.0f °/s (p99 %.0f °/s) frente a vel_limit %.0f °/s y pendiente pedida %.0f °/s\n', ...
        v_sost, prctile(v, 99), vel_lim, B.pendiente_dps(it));
end

%% FRF empírica e identificación
ifrf = find(ismember(B.tipo, ["chirp", "prbs", "carga_chirp"]))';
FRF = table(); datos = struct();
for i = ifrf
    s = M.bloque == B.bloque(i);
    [~, u, y, Ts] = uniforme(M.t_cmd_s(s), M.cmd_deg(s), M.t_s(s), M.meas_deg(s));
    datos(i).u = u; datos(i).y = y; datos(i).Ts = Ts;
    % Con fricción, una amplitud pequeña puede no despegar el volante: sin movimiento
    % no hay sistema que identificar (se anota y se salta).
    datos(i).mueve = (max(y) - min(y)) > max(4 * CUENTA, 0.2 * B.amplitud_deg(i));
    if ~datos(i).mueve
        fprintf('Bloque %d (%s, %g°): el volante casi no se movió (recorrido %.3f°). Sin FRF ni modelo.\n', ...
            B.bloque(i), B.tipo(i), B.amplitud_deg(i), max(y) - min(y));
        continue
    end
    if B.tipo(i) == "prbs"
        % PRBS: excitación estacionaria, Welch con segmentos de SEG_FRF s.
        [f, H, C] = frf_welch(u, y, Ts, round(SEG_FRF / Ts));
        fmax = F_PRBS_MAX;
    else
        % Chirp: no es estacionario. Welch promedia segmentos con y sin energía en cada
        % frecuencia y descarta el final del registro, que es donde están las
        % frecuencias altas (en la prueba con la planta falsa eso dio +30 dB falsos).
        % Se usa el registro completo y se promedia en bandas logarítmicas.
        fmax = B.f_max_hz(i);
        [f, H, C] = frf_barrido(u, y, Ts, B.f_min_hz(i), fmax);
    end
    k = f >= 0.1 & f <= fmax;
    FRF = [FRF; table(repmat(B.bloque(i), nnz(k), 1), repmat(B.tipo(i), nnz(k), 1), ...
        repmat(B.amplitud_deg(i), nnz(k), 1), repmat(campo_tabla(B, 'carga_nm', i), nnz(k), 1), f(k), ...
        20 * log10(abs(H(k))), rad2deg(unwrap(angle(H(k)))), C(k), C(k) >= COH_MIN, ...
        'VariableNames', {'bloque', 'tipo', 'amplitud_deg', 'carga_nm', 'f_hz', 'mag_db', 'fase_deg', ...
        'coherencia', 'fiable'})]; %#ok<AGROW>
end
if ~isempty(FRF), writetable(FRF, fullfile(dres, [nombre '_frf.csv'])); end

ichirp = find(ismember(B.tipo, ["chirp", "carga_chirp"]))';
iprbs = find(B.tipo == "prbs")';
iprbs = iprbs(arrayfun(@(j) numel(datos) >= j && isfield(datos, 'mueve') && isequal(datos(j).mueve, true), iprbs));
TM = table(); modelos = struct('bloque', {}, 'amplitud_deg', {}, 'carga_nm', {}, 'G1', {}, 'G2', {}, 'elegido', {});
opt = procestOptions('Display', 'off');
wgrid = 2 * pi * logspace(-2, log10(50), 3000);
for i = ichirp
    if numel(datos) < i || ~isequal(datos(i).mueve, true)
        TM = [TM; fila_vacia(B, i, "sin movimiento")]; %#ok<AGROW>
        continue
    end
    de = iddata(datos(i).y - mean(datos(i).y), datos(i).u - mean(datos(i).u), datos(i).Ts);
    dv = []; amp_v = NaN;
    if ~isempty(iprbs)
        [~, j] = min(abs(B.amplitud_deg(iprbs) - B.amplitud_deg(i)));
        jj = iprbs(j); amp_v = B.amplitud_deg(jj);
        dv = iddata(datos(jj).y - mean(datos(jj).y), datos(jj).u - mean(datos(jj).u), datos(jj).Ts);
    end
    G = cell(1, 2); estructura = {'P1D', 'P2DU'};
    fila = table();
    for q = 1:2
        try
            G{q} = procest(de, estructura{q}, opt);
        catch ME
            fprintf('Bloque %d (%s, %g°): %s falló: %s\n', B.bloque(i), B.tipo(i), B.amplitud_deg(i), ...
                estructura{q}, ME.message);
            continue
        end
        g = G{q};
        fit_v = NaN;
        if ~isempty(dv), [~, fit_v] = compare(dv, g); end
        if ~isfinite(fit_v), fit_v = NaN; end
        fit_e = g.Report.Fit.FitPercent; if ~isfinite(fit_e), fit_e = NaN; end
        Hm = squeeze(freqresp(g, wgrid));
        bw = wgrid(find(abs(Hm) < abs(g.Kp) / sqrt(2), 1)) / (2 * pi);
        if isempty(bw), bw = NaN; end
        fase = rad2deg(unwrap(angle(Hm)));
        f13 = interp1(wgrid / (2 * pi), fase, [1 3 10]);
        if q == 1
            tp = 1000 * g.Tp1; fn = NaN; zeta = NaN;
            degenerado = tp < 0.5 || tp > 5000;
        else
            tp = NaN; fn = 1 / (2 * pi * g.Tw); zeta = g.Zeta;
            % Polo fuera de la banda excitada o amortiguamiento absurdo: el chirp no
            % informa ese parámetro y el modelo no se puede usar.
            degenerado = fn > 5 * B.f_max_hz(i) || zeta > 3;
        end
        fila = [fila; table(B.bloque(i), B.tipo(i), B.amplitud_deg(i), campo_tabla(B, 'carga_nm', i), ...
            string(estructura{q}), g.Kp, tp, fn, zeta, 1000 * g.Td, fit_e, fit_v, amp_v, ...
            aic(g), bw, f13(1), f13(2), f13(3), B.f_max_hz(i), B.frac_sat_vel(i), B.frac_sat_iq(i), ...
            degenerado, false, "", ...
            'VariableNames', {'bloque', 'tipo', 'amplitud_deg', 'carga_nm', 'modelo', 'K', 'Tp1_ms', 'fn_hz', ...
            'zeta', 'Td_ms', 'ajuste_est_pct', 'ajuste_val_pct', 'amp_prbs_val_deg', 'aic', 'bw_3db_hz', ...
            'fase_1hz_deg', 'fase_3hz_deg', 'fase_10hz_deg', 'banda_max_hz', 'frac_sat_vel', 'frac_sat_iq', ...
            'degenerado', 'elegido', 'nota'})]; %#ok<AGROW>
    end
    if isempty(fila), continue; end
    validos = find(~fila.degenerado);
    if isempty(validos)
        fila.nota(:) = "ningún modelo identificable";
        TM = [TM; fila]; %#ok<AGROW>
        continue
    end
    if all(isnan(fila.ajuste_val_pct(validos))), [~, e] = min(fila.aic(validos));
    else, [~, e] = max(fila.ajuste_val_pct(validos)); end
    e = validos(e);
    fila.elegido(e) = true;
    fila.nota(fila.degenerado) = "degenerado: parámetro fuera de la banda excitada";
    TM = [TM; fila]; %#ok<AGROW>
    modelos(end + 1) = struct('bloque', B.bloque(i), 'amplitud_deg', B.amplitud_deg(i), ...
        'carga_nm', campo_tabla(B, 'carga_nm', i), 'G1', G{1}, 'G2', G{2}, 'elegido', fila.modelo(e)); %#ok<SAGROW>
end
if ~isempty(TM)
    disp(TM(:, {'tipo', 'amplitud_deg', 'carga_nm', 'modelo', 'K', 'Tp1_ms', 'fn_hz', 'zeta', 'Td_ms', ...
        'ajuste_est_pct', 'ajuste_val_pct', 'bw_3db_hz', 'banda_max_hz', 'elegido', 'nota'}))
    writetable(TM, fullfile(dres, [nombre '_modelos.csv']));
end
procedencia = struct('corrida', nombre, 'manifiesto', man, 'falso', falso, ...
    'fecha_analisis', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
save(fullfile(dres, [nombre '_modelos.mat']), 'modelos', 'TM', 'procedencia');
writetable(B, fullfile(dres, [nombre '_bloques.csv']));

%% Figuras
figs = gobjects(0);
col = lines(7);

% F1. Vista general
f1 = figure('Name', 'F1 vista general', 'Position', [50 50 1300 450]);
plot(M.t_s, M.cmd_deg, 'k', M.t_s, M.meas_deg, 'Color', col(1, :)); grid on; hold on
yl = ylim;
for i = 1:nb
    xline(B.t_ini_s(i), ':', sprintf('%s %g°', B.tipo(i), B.amplitud_deg(i)), 'Interpreter', 'none', ...
        'LabelVerticalAlignment', 'bottom', 'FontSize', 7);
end
ylim(yl); xlabel('t (s)'); ylabel('\theta volante (°)'); legend('\theta_{cmd}', '\theta_{meas}');
title(['Caracterización formal · ' strrep(nombre, '_', '\_')]);
figs(end + 1) = f1;

% F2. Escalones normalizados por amplitud y sentido
if ~isempty(TE)
    amps = unique(TE.amplitud_deg)';
    f2 = figure('Name', 'F2 escalones', 'Position', [50 50 1200 420]);
    tiledlayout(1, numel(amps), 'TileSpacing', 'compact');
    for a = amps
        nexttile; hold on; grid on
        for i = find(TE.amplitud_deg == a)'
            s = M.t_s >= E.t_cmd_s(i) & M.t_s < E.t_cmd_s(i) + sost;
            previo = M.t_s < E.t_cmd_s(i) & M.t_s >= E.t_cmd_s(i) - 0.05;
            y = dirn(i) * (M.meas_deg(s) - mean(M.meas_deg(previo))) / A(i);
            plot(1000 * (M.t_s(s) - E.t_cmd_s(i)), y, 'Color', col(1 + (dirn(i) > 0), :));
        end
        yline(1, 'k:'); xlabel('t (ms)'); ylabel('respuesta / A'); xlim([0 600]);
        title(sprintf('A = %g° (azul izq., rojo der.)', a));
    end
    figs(end + 1) = f2;
end

% F3. FRF empírica por bloque
if ~isempty(FRF)
    f3 = figure('Name', 'F3 FRF empírica', 'Position', [50 50 1000 800]);
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact');
    ax = gobjects(3, 1); for q = 1:3, ax(q) = nexttile; hold on; grid on; set(gca, 'XScale', 'log'); end
    bl = unique(FRF.bloque)'; leyenda = strings(0);
    for q = 1:numel(bl)
        r = FRF.bloque == bl(q);
        estilo = '-'; if FRF.tipo(find(r, 1)) == "prbs", estilo = '--'; end
        plot(ax(1), FRF.f_hz(r), FRF.mag_db(r), estilo);
        plot(ax(2), FRF.f_hz(r), FRF.fase_deg(r), estilo);
        plot(ax(3), FRF.f_hz(r), FRF.coherencia(r), estilo);
        leyenda(end + 1) = sprintf('%s %g°', FRF.tipo(find(r, 1)), FRF.amplitud_deg(find(r, 1))); %#ok<SAGROW>
    end
    yline(ax(1), -3, 'k:'); ylabel(ax(1), '|H| (dB)'); ylabel(ax(2), 'fase (°)');
    yline(ax(3), COH_MIN, 'k:'); ylabel(ax(3), 'coherencia (local en el chirp)'); xlabel(ax(3), 'f (Hz)');
    legend(ax(1), leyenda, 'Location', 'southwest', 'Interpreter', 'none');
    linkaxes(ax, 'x'); title(tl, 'FRF empírica \theta_{meas}/\theta_{cmd} (H1, Welch)');
    figs(end + 1) = f3;
end

% F4. FRF empírica frente a G1 y G2, por amplitud de chirp
if ~isempty(modelos)
    f4 = figure('Name', 'F4 modelos', 'Position', [50 50 1300 600]);
    tiledlayout(2, numel(modelos), 'TileSpacing', 'compact');
    for q = 1:numel(modelos)
        r = FRF.bloque == modelos(q).bloque;
        for fila = 1:2
            nexttile(q + (fila - 1) * numel(modelos)); hold on; grid on; set(gca, 'XScale', 'log');
            if fila == 1
                plot(FRF.f_hz(r), FRF.mag_db(r), '.', 'Color', [0.6 0.6 0.6]);
            else
                plot(FRF.f_hz(r), FRF.fase_deg(r), '.', 'Color', [0.6 0.6 0.6]);
            end
            for g = {modelos(q).G1, modelos(q).G2}
                if isempty(g{1}), continue; end
                Hm = squeeze(freqresp(g{1}, wgrid));
                if fila == 1, plot(wgrid / (2 * pi), 20 * log10(abs(Hm)), 'LineWidth', 1.2);
                else, plot(wgrid / (2 * pi), rad2deg(unwrap(angle(Hm))), 'LineWidth', 1.2); end
            end
            xlim([0.1 max(FRF.f_hz(r))]);
            if fila == 1
                title(sprintf('%g°%s · elegido %s', modelos(q).amplitud_deg, ...
                    texto_carga(modelos(q).carga_nm), modelos(q).elegido)); ylabel('|H| (dB)');
            else
                ylabel('fase (°)'); xlabel('f (Hz)');
            end
        end
    end
    legend('FRF empírica', 'G1 (P1D)', 'G2 (P2DU)', 'Location', 'southwest');
    figs(end + 1) = f4;
end

% F5. Histéresis en las rampas
ir = find(B.tipo == "rampa")';
if ~isempty(ir)
    f5 = figure('Name', 'F5 histéresis', 'Position', [50 50 1100 450]);
    tiledlayout(1, 2, 'TileSpacing', 'compact');
    nexttile; hold on; grid on
    for i = ir
        s = M.bloque == B.bloque(i);
        plot(M.cmd_deg(s), M.cmd_deg(s) - M.meas_deg(s));
    end
    xlabel('\theta_{cmd} (°)'); ylabel('\theta_{cmd} − \theta_{meas} (°)');
    legend(compose('%g °/s', B.vel_dps(ir)), 'Location', 'best'); title('Error en las rampas');
    nexttile; hold on; grid on
    for i = ir
        s = M.bloque == B.bloque(i);
        plot(M.t_s(s) - B.t_ini_s(i), M.iq_a(s));
    end
    xlabel('t desde el inicio de la rampa (s)'); ylabel('I_q (A)'); title('Corriente en las rampas');
    figs(end + 1) = f5;
end

% F6. Tasa
if ~isempty(it)
    s = M.bloque == B.bloque(it);
    f6 = figure('Name', 'F6 tasa', 'Position', [50 50 1000 420]);
    plot(M.t_s(s), M.omega_dps(s)); grid on; hold on
    yline([-vel_lim vel_lim], 'k--', 'vel\_limit');
    xlabel('t (s)'); ylabel('\omega (°/s)');
    title(sprintf('Prueba de tasa: pendiente pedida %.0f °/s, sostenida %.0f °/s', B.pendiente_dps(it), v_sost));
    figs(end + 1) = f6;
end

% F7. Validación en el PRBS con el modelo elegido de la amplitud más cercana
if ~isempty(iprbs) && ~isempty(modelos)
    f7 = figure('Name', 'F7 validación', 'Position', [50 50 1200 300 * numel(iprbs)]);
    tiledlayout(numel(iprbs), 1, 'TileSpacing', 'compact');
    sin_carga = isnan([modelos.carga_nm]);
    for jj = iprbs
        nexttile; hold on; grid on
        cand = find(sin_carga);
        if isempty(cand), continue; end
        [~, q] = min(abs([modelos(cand).amplitud_deg] - B.amplitud_deg(jj)));
        mq = modelos(cand(q));
        if mq.elegido == "P1D", g = mq.G1; else, g = mq.G2; end
        dv = iddata(datos(jj).y - mean(datos(jj).y), datos(jj).u - mean(datos(jj).u), datos(jj).Ts);
        ysim = sim(g, dv);
        t = (0:numel(datos(jj).y) - 1)' * datos(jj).Ts;
        k = t >= 5 & t < 8;
        plot(t(k), dv.u(k), 'k:', t(k), dv.y(k), t(k), ysim.y(k), 'LineWidth', 1);
        legend('\theta_{cmd}', '\theta_{meas}', sprintf('modelo %s de %g°', mq.elegido, mq.amplitud_deg));
        xlabel('t (s)'); ylabel('\theta (°)'); title(sprintf('PRBS %g° (3 s de muestra)', B.amplitud_deg(jj)));
    end
    figs(end + 1) = f7;
end

for j = 1:numel(figs)
    exportgraphics(figs(j), fullfile(dfig, sprintf('F%d.png', j)), 'Resolution', 200);
    savefig(figs(j), fullfile(dfig, sprintf('F%d.fig', j)));
end
fprintf('Resultados en %s\nFiguras en %s\nLog en %s\n', dres, dfig, flog);
diary('off');

function tp = primero(t, cond)
    k = find(cond, 1);
    if isempty(k), tp = NaN; else, tp = t(k); end
end

function f = fila_vacia(B, i, nota)
    f = table(B.bloque(i), B.tipo(i), B.amplitud_deg(i), campo_tabla(B, 'carga_nm', i), "ninguno", ...
        NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, B.f_max_hz(i), ...
        B.frac_sat_vel(i), B.frac_sat_iq(i), true, false, string(nota), ...
        'VariableNames', {'bloque', 'tipo', 'amplitud_deg', 'carga_nm', 'modelo', 'K', 'Tp1_ms', 'fn_hz', ...
        'zeta', 'Td_ms', 'ajuste_est_pct', 'ajuste_val_pct', 'amp_prbs_val_deg', 'aic', 'bw_3db_hz', ...
        'fase_1hz_deg', 'fase_3hz_deg', 'fase_10hz_deg', 'banda_max_hz', 'frac_sat_vel', 'frac_sat_iq', ...
        'degenerado', 'elegido', 'nota'});
end

function v = campo(s, nombre, defecto)
    if isstruct(s) && isfield(s, nombre) && ~isempty(s.(nombre)), v = s.(nombre); else, v = defecto; end
end

function v = campo_tabla(T, nombre, i)
    if ismember(nombre, T.Properties.VariableNames), v = T.(nombre)(i); else, v = NaN; end
end

function s = texto(x)
    if isempty(x), s = 'ninguno'; else, s = char(string(x)); end
end

function s = texto_carga(c)
    if isnan(c), s = ''; else, s = sprintf(', carga %g N·m', c); end
end

function [tg, u, y, Ts] = uniforme(t_cmd, cmd, t_meas, meas)
    % Paso uniforme: la consigna se mantiene hasta el siguiente envío (retén de orden cero)
    % y la medida se interpola linealmente.
    Ts = median(diff(t_meas));
    tg = (max(t_cmd(1), t_meas(1)):Ts:min(t_cmd(end), t_meas(end)))';
    u = interp1(t_cmd, cmd, tg, 'previous');
    y = interp1(t_meas, meas, tg, 'linear');
end

function [fb, Hb, Cb] = frf_barrido(u, y, Ts, fmin, fmax)
    % FRF de un barrido (chirp) con el registro completo y ventana de Tukey (10 %).
    % En cada banda logarítmica (40 por década): H = Σ Y·U* / Σ |U|² y coherencia
    % local = |Σ Y·U*|² / (Σ |U|² · Σ |Y|²), que baja con ruido o no linealidad.
    u = u - mean(u); y = y - mean(y);
    N = numel(u); r = 0.1; m = floor(r * N / 2);
    w = ones(N, 1); rampa = 0.5 - 0.5 * cos(pi * (0:m - 1)' / m);
    w(1:m) = rampa; w(end - m + 1:end) = flipud(rampa);
    U = fft(w .* u); Y = fft(w .* y);
    nf = floor(N / 2) + 1; f = (0:nf - 1)' / (N * Ts);
    U = U(1:nf); Y = Y(1:nf);
    bordes = logspace(log10(fmin), log10(fmax), max(2, round(40 * log10(fmax / fmin))) + 1);
    nb = numel(bordes) - 1;
    [fb, Cb] = deal(nan(nb, 1)); Hb = complex(nan(nb, 1));
    for b = 1:nb
        k = f >= bordes(b) & f < bordes(b + 1);
        if ~any(k), continue; end
        Suy = sum(Y(k) .* conj(U(k))); Suu = sum(abs(U(k)) .^ 2); Syy = sum(abs(Y(k)) .^ 2);
        fb(b) = sqrt(bordes(b) * bordes(b + 1));
        Hb(b) = Suy / Suu;
        Cb(b) = abs(Suy) ^ 2 / (Suu * Syy);
    end
    ok = isfinite(fb); fb = fb(ok); Hb = Hb(ok); Cb = Cb(ok);
end

function [f, H, C] = frf_welch(u, y, Ts, L)
    % FRF H1 = S_uy/S_uu y coherencia con segmentos de Hann solapados al 50 %.
    u = u - mean(u); y = y - mean(y);
    N = numel(u); L = min(N, L); paso = floor(L / 2);
    w = 0.5 - 0.5 * cos(2 * pi * (0:L - 1)' / (L - 1));
    Suu = 0; Syy = 0; Suy = 0;
    for i0 = 1:paso:(N - L + 1)
        U = fft(w .* u(i0:i0 + L - 1)); Y = fft(w .* y(i0:i0 + L - 1));
        Suu = Suu + abs(U) .^ 2; Syy = Syy + abs(Y) .^ 2; Suy = Suy + conj(U) .* Y;
    end
    nf = floor(L / 2) + 1;
    f = (0:nf - 1)' / (L * Ts);
    H = Suy(1:nf) ./ Suu(1:nf);
    C = abs(Suy(1:nf)) .^ 2 ./ (Suu(1:nf) .* Syy(1:nf));
end
