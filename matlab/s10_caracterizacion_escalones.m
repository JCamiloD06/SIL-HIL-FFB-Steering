%% s10_caracterizacion_escalones.m
% Caracterización del actuador (plan §9.1): escalones pequeños y rampa lenta.
% Datos: data/raw/caracterizacion/<fecha>_escalones/ (scripts/acq/c01_escalones_pequenos.py)
% Inferencia: DESCRIPTIVA. Estado: EXPLORATORIO (protocolo de caracterización aún no congelado).
%
% Por escalón (ángulos de volante, grados):
%   t_mov  primer instante en que el volante avanza max(0.2·A, 1 cuenta = 0.125°) hacia el objetivo
%   t50    primer instante en que llega al 50 % del escalón (NaN si no llega en 1 s)
%   t90    igual, al 90 %
%   e_fin  error final con signo, media de los últimos 200 ms (negativo = se quedó corto)
%   cambio_sentido  el escalón va en sentido contrario al anterior (holgura + fricción) o no
% Rampa: diferencia cmd - meas subiendo frente a bajando = ancho del lazo de histéresis.
% Indicadores por corrida (<corrida>_indicadores.csv) para comparar sintonías con s11.
% Corriente (si la corrida la registró): Iq al despegar en la rampa = máximo |Iq| en los
% últimos 50 ms de cada tramo pegado (|ω| < 2 °/s y |error| > 1 cuenta durante > 100 ms).
%
% Uso:  matlab -batch "run('scripts/s10_caracterizacion_escalones.m')"
%       (toma la carpeta más reciente; para otra, definir antes la variable 'carpeta')

if ~exist('raiz', 'var')
    raiz = fileparts(fileparts(mfilename('fullpath')));
end
if ~exist('carpeta', 'var')
    d = dir(fullfile(raiz, 'data', 'raw', 'caracterizacion', '*_escalones*'));
    assert(~isempty(d), 'No hay corridas en data/raw/caracterizacion');
    [~, k] = max(datenum({d.date})); %#ok<DATNM>
    carpeta = fullfile(d(k).folder, d(k).name);
end
if ~isfolder(carpeta), carpeta = fullfile(raiz, carpeta); end   % ruta relativa a la raíz del paper
[~, nombre] = fileparts(carpeta);
fprintf('Corrida: %s\n', nombre);

CUENTA = 360 / 2880;      % resolución del encoder en grados de volante
HOLD_FIN = 0.2;           % ventana para el error final (s)
VENTANA = 1.0;            % duración de cada escalón (s), la del script de adquisición

M = readtable(fullfile(carpeta, 'muestras.csv'), 'TextType', 'string');
E = readtable(fullfile(carpeta, 'escalones.csv'));
man = jsondecode(fileread(fullfile(carpeta, 'manifiesto.json')));
if isfield(man.placa, 'FALSO'), warning('DATOS FALSOS DE PRUEBA'); end
fprintf('%d muestras, %d escalones, dt mediano %.2f ms, corte: %s\n', height(M), height(E), ...
    1000 * median(diff(M.t_s)), string(man.corte));

%% Métricas por escalón
n = height(E);
dirn = sign(E.hacia_deg - E.desde_deg);
A = abs(E.hacia_deg - E.desde_deg);
t_mov = nan(n, 1); t50 = t_mov; t90 = t_mov; e_fin = t_mov; sobrepaso = t_mov;
cambio = nan(n, 1);  % 1 = cambio de sentido, 0 = mismo sentido, NaN = sin historia
for i = 1:n
    s = M.escalon == E.escalon(i) & M.fase == "escalon";
    t = M.t_s(s) - E.t_cmd_s(i);
    previo = M.t_s < E.t_cmd_s(i) & M.t_s >= E.t_cmd_s(i) - 0.05;
    th0 = mean(M.meas_deg(previo));
    y = dirn(i) * (M.meas_deg(s) - th0);
    t_mov(i) = primero(t, y >= max(0.2 * A(i), CUENTA));
    t50(i) = primero(t, y >= 0.5 * A(i));
    t90(i) = primero(t, y >= 0.9 * A(i));
    m = M.meas_deg(s);
    e_fin(i) = dirn(i) * (mean(m(t >= t(end) - HOLD_FIN)) - E.hacia_deg(i));
    sobrepaso(i) = max(0, max(y) - A(i));
    if i > 1 && dirn(i - 1) ~= 0
        cambio(i) = double(dirn(i) ~= dirn(i - 1));
    end
end
T = table(E.escalon, E.repeticion, E.desde_deg, E.hacia_deg, A, dirn, cambio, ...
    t_mov, t50, t90, e_fin, sobrepaso, 'VariableNames', {'escalon', 'repeticion', 'desde_deg', ...
    'hacia_deg', 'amplitud_deg', 'sentido', 'cambio_sentido', 't_mov_s', 't50_s', 't90_s', ...
    'e_fin_deg', 'sobrepaso_deg'});

%% Resumen por amplitud y tipo de escalón
amps = unique(A);
filas = {};
for a = amps'
    for c = [1 0]
        k = A == a & cambio == c;
        if ~any(k), continue; end
        filas(end + 1, :) = {a, c, nnz(k), mean(~isnan(t50(k))), mean(t50(k) <= 0.05), ...
            mean(t50(k) <= 0.10), median(t_mov(k), 'omitnan'), median(t50(k), 'omitnan'), ...
            median(t90(k), 'omitnan'), median(abs(e_fin(k))), max(abs(e_fin(k)))}; %#ok<SAGROW>
    end
end
R = cell2table(filas, 'VariableNames', {'amplitud_deg', 'cambio_sentido', 'n', ...
    'frac_llega50_1s', 'frac_llega50_50ms', 'frac_llega50_100ms', 'med_t_mov_s', ...
    'med_t50_s', 'med_t90_s', 'med_abs_e_fin_deg', 'max_abs_e_fin_deg'});
disp(R)

%% Rampa: histéresis
r = M.fase == "rampa";
tr = M.t_s(r); cr = M.cmd_deg(r); mr = M.meas_deg(r);
dc = gradient(cr, tr);
sube = dc > 0.5; baja = dc < -0.5;
err = cr - mr;
hist_ancho = median(err(sube)) - median(err(baja));
% "Pegado": fracción de ventanas de 100 ms (dos ciclos del controlador) en que
% la referencia avanzó más de una cuenta pero el volante no se movió ni una.
% A 2 °/s la referencia avanza 0,1° en 50 ms, menos de una cuenta: por eso 100 ms.
w = 0.10; quieto = [];
for t0 = tr(1):w:tr(end) - w
    k = tr >= t0 & tr < t0 + w;
    if nnz(k) < 2, continue; end
    if abs(cr(find(k, 1, 'last')) - cr(find(k, 1))) > CUENTA
        quieto(end + 1) = abs(mr(find(k, 1, 'last')) - mr(find(k, 1))) < CUENTA / 2; %#ok<SAGROW>
    end
end
fprintf('Rampa: error mediano subiendo %+.3f°, bajando %+.3f°, ancho de histéresis %.3f°\n', ...
    median(err(sube)), median(err(baja)), hist_ancho);
fprintf('Rampa: volante quieto en %.0f %% de las ventanas de 100 ms con referencia en movimiento (n = %d)\n', ...
    100 * mean(quieto), numel(quieto));
fprintf('Rampa: error máximo %.2f°, error absoluto mediano %.2f°\n', max(abs(err)), median(abs(err)));

%% Salidas
dres = fullfile(raiz, 'results', 'exploratory', 'caracterizacion');
dfig = fullfile(raiz, 'figures', 'draft', 'caracterizacion', nombre);
if ~exist(dres, 'dir'), mkdir(dres); end
if ~exist(dfig, 'dir'), mkdir(dfig); end
writetable(T, fullfile(dres, [nombre '_metricas.csv']));
writetable(R, fullfile(dres, [nombre '_resumen.csv']));
fid = fopen(fullfile(dres, [nombre '_rampa.txt']), 'w');
fprintf(fid, 'corrida %s\nancho_histeresis_deg %.4f\nerr_mediano_subiendo_deg %.4f\nerr_mediano_bajando_deg %.4f\nfrac_quieto_100ms %.4f\nn_ventanas %d\n', ...
    nombre, hist_ancho, median(err(sube)), median(err(baja)), mean(quieto), numel(quieto));
fclose(fid);

%% Indicadores de la corrida (una fila, para comparar sintonías)
g = struct('pos_gain', NaN, 'vel_gain', NaN, 'vel_int_gain', NaN);
if isfield(man, 'ganancias_usadas')
    g = man.ganancias_usadas;
elseif isfield(man.placa, 'axis0_controller_config_pos_gain')
    g.pos_gain = man.placa.axis0_controller_config_pos_gain;
    g.vel_gain = man.placa.axis0_controller_config_vel_gain;
    g.vel_int_gain = man.placa.axis0_controller_config_vel_integrator_gain;
end
re = M.fase == "reposo" & M.t_s > M.t_s(find(M.fase == "reposo", 1)) + 0.5;
k12 = A >= 1 & A <= 2;
corte_txt = "ninguno"; if ~isempty(man.corte), corte_txt = string(man.corte); end
iq_desp = NaN; iq_quieto = NaN; tiene_iq = ismember('iq_a', M.Properties.VariableNames) && any(isfinite(M.iq_a));
if tiene_iq
    iqr_ = M.iq_a(r); wr_ = M.omega_dps(r);
    pegado = abs(wr_) < 2 & abs(err) > CUENTA;
    d = diff([0; pegado; 0]); ini = find(d == 1); fin = find(d == -1) - 1;
    picos = [];
    for j = 1:numel(ini)
        if tr(fin(j)) - tr(ini(j)) < 0.1, continue; end
        kk = ini(j):fin(j); kk = kk(tr(kk) >= tr(fin(j)) - 0.05);
        picos(end + 1) = max(abs(iqr_(kk))); %#ok<SAGROW>
    end
    iq_desp = median(picos); iq_quieto = median(abs(iqr_(pegado)));
    fprintf('Corriente: |Iq| al despegar (mediana de %d tramos) %.2f A; |Iq| mediana pegado %.2f A\n', ...
        numel(picos), iq_desp, iq_quieto);
end
I = table(string(nombre), g.pos_gain, g.vel_gain, g.vel_int_gain, ...
    mean(~isnan(t50(k12))), mean(~isnan(t50(k12 & cambio == 1))), mean(~isnan(t50(A <= 0.5))), ...
    1000 * median(t50(A == 8), 'omitnan'), 100 * median(sobrepaso(A == 8)) / 8, ...
    median(abs(e_fin(A >= 4))), median(abs(err)), max(abs(err)), mean(quieto), ...
    max(M.meas_deg(re)) - min(M.meas_deg(re)), iq_desp, iq_quieto, corte_txt, ...
    'VariableNames', {'corrida', 'pos_gain', 'vel_gain', 'vel_int_gain', ...
    'frac50_1s_A1a2', 'frac50_1s_A1a2_cambio', 'frac50_1s_A025a05', 't50_8deg_ms', ...
    'sobrepaso_8deg_pct', 'abs_efin_A4a8_deg', 'rampa_abs_err_med_deg', 'rampa_abs_err_max_deg', ...
    'rampa_frac_quieto', 'reposo_pp_deg', 'iq_despegue_a', 'iq_pegado_a', 'corte'});
writetable(I, fullfile(dres, [nombre '_indicadores.csv']));
disp(I)

%% Figuras
col = lines(2);   % 1 = cambio de sentido, 2 = mismo sentido

% F1. Vista general
f1 = figure('Name', 'F1 vista general', 'Position', [50 50 1200 450]);
plot(M.t_s, M.cmd_deg, 'k', M.t_s, M.meas_deg, 'Color', col(1, :)); grid on
xlabel('t (s)'); ylabel('\theta volante (°)'); legend('\theta_{cmd}', '\theta_{meas}');
title(['Escalones y rampa · ' strrep(nombre, '_', '\_')]);

% F2. Respuestas normalizadas por amplitud
f2 = figure('Name', 'F2 respuestas normalizadas', 'Position', [50 50 1200 700]);
tl = tiledlayout(2, ceil(numel(amps) / 2), 'TileSpacing', 'compact');
for j = 1:numel(amps)
    nexttile; hold on; grid on
    for i = find(A == amps(j))'
        s = M.escalon == E.escalon(i) & M.fase == "escalon";
        previo = M.t_s < E.t_cmd_s(i) & M.t_s >= E.t_cmd_s(i) - 0.05;
        y = dirn(i) * (M.meas_deg(s) - mean(M.meas_deg(previo))) / A(i);
        c = col(1 + (cambio(i) ~= 1), :);
        if isnan(cambio(i)), c = [.6 .6 .6]; end
        plot(M.t_s(s) - E.t_cmd_s(i), y, 'Color', [c 0.6]);
    end
    yline([0.5 1], ':'); xline(0.05, '--', '50 ms');
    title(sprintf('A = %.2f°', amps(j))); xlim([0 VENTANA]); ylim([-0.5 1.8]);
end
xlabel(tl, 't desde el cambio de consigna (s)'); ylabel(tl, 'avance / A');
title(tl, 'Azul: cambio de sentido · Naranja: mismo sentido');

% F3. Tiempo al 50 % frente a amplitud
f3 = figure('Name', 'F3 t50 vs amplitud', 'Position', [50 50 1100 420]);
tiledlayout(1, 2, 'TileSpacing', 'compact');
nexttile; hold on; grid on
for c = [1 0]
    k = cambio == c;
    jit = A(k) .* (1 + 0.06 * (c - 0.5));
    scatter(jit, 1000 * t50(k), 25, col(2 - c, :), 'filled', 'MarkerFaceAlpha', 0.6);
    nk = k & isnan(t50);
    scatter(A(nk) .* (1 + 0.06 * (c - 0.5)), repmat(1000 * VENTANA * 1.1, nnz(nk), 1), 40, col(2 - c, :), 'x');
end
set(gca, 'XScale', 'log', 'YScale', 'log', 'XTick', amps); yline(50, '--', '1 ciclo (50 ms)');
xlabel('Amplitud del escalón (° de volante)'); ylabel('t_{50} (ms)  ·  x = no llegó en 1 s');
legend('cambio de sentido', '', 'mismo sentido', '', 'Location', 'northeast');
nexttile; hold on; grid on
for c = [1 0]
    Rc = R(R.cambio_sentido == c, :);
    plot(Rc.amplitud_deg, 100 * Rc.frac_llega50_50ms, '-o', 'Color', col(2 - c, :));
    plot(Rc.amplitud_deg, 100 * Rc.frac_llega50_1s, ':s', 'Color', col(2 - c, :));
end
set(gca, 'XScale', 'log', 'XTick', amps); ylim([0 105]);
xlabel('Amplitud del escalón (° de volante)'); ylabel('% de escalones que llegan al 50 %');
legend('cambio, en 50 ms', 'cambio, en 1 s', 'mismo, en 50 ms', 'mismo, en 1 s', 'Location', 'southeast');

% F4. Error final
f4 = figure('Name', 'F4 error final', 'Position', [50 50 600 420]); hold on; grid on
for c = [1 0]
    k = cambio == c;
    scatter(A(k) .* (1 + 0.06 * (c - 0.5)), e_fin(k), 25, col(2 - c, :), 'filled', 'MarkerFaceAlpha', 0.6);
end
set(gca, 'XScale', 'log', 'XTick', amps); yline([-CUENTA CUENTA], ':', {'-1 cuenta', '+1 cuenta'});
xlabel('Amplitud (° de volante)'); ylabel('e_{fin} (°) · negativo = se quedó corto');
legend('cambio de sentido', 'mismo sentido');

% F5. Rampa: histéresis
f5 = figure('Name', 'F5 rampa', 'Position', [50 50 1100 420]);
tiledlayout(1, 2, 'TileSpacing', 'compact');
nexttile; plot(cr, mr, 'Color', col(1, :)); hold on; plot(cr, cr, 'k:'); grid on; axis equal
xlabel('\theta_{cmd} (°)'); ylabel('\theta_{meas} (°)'); title('Lazo de histéresis');
nexttile; plot(tr - tr(1), err, 'Color', col(1, :)); grid on
xlabel('t (s)'); ylabel('\theta_{cmd} - \theta_{meas} (°)');
title(sprintf('Ancho de histéresis %.2f°', hist_ancho));

figs = [f1 f2 f3 f4 f5];
if tiene_iq
    % F6. Rampa con corriente e integrador: ¿cuánta corriente hace falta para despegar?
    f6 = figure('Name', 'F6 rampa y corriente', 'Position', [50 50 1200 650]);
    tl6 = tiledlayout(2, 1, 'TileSpacing', 'compact');
    ax1 = nexttile; plot(tr - tr(1), cr, 'k', tr - tr(1), mr, 'Color', col(1, :)); grid on
    ylabel('\theta (°)'); legend('\theta_{cmd}', '\theta_{meas}');
    ax2 = nexttile; plot(tr - tr(1), M.iq_a(r), 'Color', col(2, :)); hold on; grid on
    ig = M.integ_nm(r); ki = isfinite(ig);
    if any(ki)
        yyaxis right; plot(tr(ki) - tr(1), ig(ki), '.', 'MarkerSize', 4); ylabel('integrador (N·m)'); yyaxis left
    end
    ylabel('I_q (A)'); xlabel('t (s)'); linkaxes([ax1 ax2], 'x');
    title(tl6, 'Rampa lenta: posición arriba, corriente abajo');
    figs(end + 1) = f6;
end
for j = 1:numel(figs)
    exportgraphics(figs(j), fullfile(dfig, sprintf('F%d.png', j)), 'Resolution', 200);
    savefig(figs(j), fullfile(dfig, sprintf('F%d.fig', j)));
end
fprintf('Resultados en %s\nFiguras en %s\n', dres, dfig);

function tp = primero(t, cond)
    k = find(cond, 1);
    if isempty(k), tp = NaN; else, tp = t(k); end
end
