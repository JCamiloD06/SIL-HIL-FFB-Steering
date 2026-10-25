%% s17_ffb_ac.m
% ¿Sirve finalFF de Assetto Corsa como fuente de la carga de HIL-L? (Q2-D-034)
% Datos: data/raw/trialsteer/<corrida>/ac_ffb.csv (scripts/acq/hil/registro_ffb_ac.py, ~100 Hz),
% en SIL o en HIL-U. Toma la corrida más reciente que tenga ac_ffb.csv; para otra, definir
% antes la variable 'carpeta'.
% Inferencia: DESCRIPTIVA. Estado: EXPLORATORIO. Solo MATLAB base.
%
% Contexto: TrialSteer instala en AC un controls.ini con vJoy como volante, LOCK=900 y
% FF_GAIN=0.95, así que AC calcula FFB. finalFF está en la parte ampliada de la página de
% física, cuyo orden de campos NO ESTÁ VERIFICADO en AC 1.16.4: si el desplazamiento está mal,
% se lee otro campo. Este script busca las huellas de una señal de FFB real.
%
% Criterios (todos necesarios para usar finalFF como fuente de la carga):
%   C1  AC estaba corriendo: packetId cambia (si no, la memoria eran ceros).
%   C2  finalFF no es constante y está acotada (|finalFF| ≤ ~1,5; AC la entrega normalizada).
%   C3  Con el carro detenido (< 1 km/h) y el volante recto, |finalFF| es pequeña.
%   C4  En curva, finalFF tiene signo coherente con la aceleración lateral y crece con ella
%       (correlación de magnitud > 0,5 con |a_y|, a_y = 9,81·acc_g_x, convención de DECISIONS.md).
%
% Salidas: logs/s17_<corrida>.log, figures/draft/ffb_ac/<corrida>/F1–F3 (.png y .fig)
% Uso:  matlab -batch "run('scripts/s17_ffb_ac.m')"
%
% Guía de observación para el investigador (AGENTS.md §17)
%   F1  finalFF, a_y y velocidad a lo largo de la vuelta. ¿Se mueve finalFF en las curvas y
%       vuelve a cero en las rectas? ¿Tiene picos en los pianos (CURBS = 0,4 en controls.ini)?
%   F2  finalFF frente a a_y. ¿Una nube con pendiente clara (alineante) o una nube sin forma
%       (campo equivocado)? ¿Se aplana en ±1 (saturación del FFB de AC)? ¿Qué signo tiene la
%       pendiente? El signo decide el de K_FFB.
%   F3  Espectro de finalFF. ¿Hay energía por encima de 10 Hz (efectos de pianos o vibración)
%       que el motor no deba reproducir? Sirve para fijar el filtro de la carga.
%   Preguntas: ¿pasan C1–C4? ¿La escala de finalFF permite elegir K_FFB dentro del torque que
%   soporta el motor sin saturar el lazo de posición?

clearvars -except carpeta
raiz = fileparts(fileparts(mfilename('fullpath')));
if isempty(raiz), raiz = pwd; end
if ~exist('carpeta', 'var')
    d = dir(fullfile(raiz, 'data', 'raw', 'trialsteer', '*', 'ac_ffb.csv'));
    if isempty(d), error('No hay corridas con ac_ffb.csv en data/raw/trialsteer.'); end
    [~, k] = max([d.datenum]);
    carpeta = d(k).folder;
end
if ~isfolder(carpeta), carpeta = fullfile(raiz, carpeta); end   % ruta relativa a la raíz del paper
[~, nombre] = fileparts(carpeta);
dfig = fullfile(raiz, 'figures', 'draft', 'ffb_ac', nombre);
if ~exist(dfig, 'dir'), mkdir(dfig); end
if ~exist(fullfile(raiz, 'logs'), 'dir'), mkdir(fullfile(raiz, 'logs')); end
diary(fullfile(raiz, 'logs', ['s17_' nombre '.log'])); diary on
fprintf('s17_ffb_ac  %s\nCorrida: %s\n', datestr(now, 31), carpeta);

T = readtable(fullfile(carpeta, 'ac_ffb.csv'));
[~, iu] = unique(T.packet_id, 'stable');       % una fila por paquete de física
T = T(sort(iu), :);
n_paq = height(T);
t = T.t_perf_s - T.t_perf_s(1);
ff = T.final_ff; v = T.speed_kmh; ay = 9.81 * T.acc_g_x;

c1 = n_paq > 100;
fprintf('\nC1  paquetes distintos: %d -> %s\n', n_paq, pasa(c1));
if ~c1
    fprintf('    AC no estaba corriendo o la memoria no se actualizó. No se puede evaluar.\n');
    diary off; return
end
c2 = std(ff, 'omitnan') > 1e-3 && max(abs(ff)) <= 1.5;
fprintf('C2  finalFF: rango [%.3f, %.3f], desviación %.3f, %.1f %% de muestras en |ff| ≥ 0,99 -> %s\n', ...
    min(ff), max(ff), std(ff, 'omitnan'), 100 * mean(abs(ff) >= 0.99), pasa(c2));
quieto = v < 1 & abs(T.steer_angle) < 0.02;
if any(quieto)
    c3 = median(abs(ff(quieto))) < 0.05;
    fprintf('C3  detenido y recto (%d muestras): mediana |ff| = %.3f -> %s\n', nnz(quieto), median(abs(ff(quieto))), pasa(c3));
else
    c3 = NaN; fprintf('C3  sin muestras con el carro detenido y recto: no evaluable\n');
end
curva = v > 20 & abs(ay) > 2;
if nnz(curva) > 50
    R = corrcoef(ay(curva), ff(curva)); r_signo = R(1, 2);
    Rm = corrcoef(abs(ay(curva)), abs(ff(curva))); r_mag = Rm(1, 2);
    p = polyfit(ay(curva), ff(curva), 1);
    c4 = abs(r_signo) > 0.5 && r_mag > 0.5;
    fprintf('C4  en curva (%d muestras): r(a_y, ff) = %+.2f, r(|a_y|, |ff|) = %.2f, pendiente %+.4f por m/s² -> %s\n', ...
        nnz(curva), r_signo, r_mag, p(1), pasa(c4));
    % La carga alineante debe tener el signo de signo_vjoy·a_y (carga_hil.py; signo_vjoy = +1 en
    % TrialSteer). Con ff ≈ p·a_y, τ = k·signo_ff·ff tiene ese signo si signo_ff = sign(p).
    fprintf('    Para HIL-L con finalFF: --carga-signo-ff %+d. Escala: |finalFF| ≈ %.3f por m/s² de a_y\n', ...
        sign(p(1)), abs(p(1)));
else
    c4 = NaN; fprintf('C4  pocas muestras en curva (v > 20 km/h y |a_y| > 2 m/s²): no evaluable\n');
end

% F1
f1 = figure('Name', 'F1 finalFF en la vuelta', 'Position', [50 50 1100 650]);
subplot(3, 1, 1); plot(t, ff); ylabel('finalFF'); grid on; title(['finalFF de AC, ' strrep(nombre, '_', '\_')]);
subplot(3, 1, 2); plot(t, ay); ylabel('a_y (m/s^2)'); grid on;
subplot(3, 1, 3); plot(t, v); ylabel('v (km/h)'); xlabel('t (s)'); grid on;
guardar(f1, dfig, 'F1_vuelta');
% F2
f2 = figure('Name', 'F2 finalFF frente a a_y', 'Position', [80 80 650 550]);
scatter(ay(v > 20), ff(v > 20), 4, v(v > 20), 'filled'); cb = colorbar; cb.Label.String = 'v (km/h)';
xlabel('a_y = 9,81·acc\_g\_x (m/s^2)'); ylabel('finalFF'); grid on; title('finalFF frente a aceleración lateral (v > 20 km/h)');
guardar(f2, dfig, 'F2_ff_vs_ay');
% F3: espectro con segmentos de Welch hechos a mano (sin toolbox), sobre la señal remuestreada
fs = 100; ti = (t(1):1/fs:t(end))'; ffi = interp1(t, ff, ti, 'linear');
nseg = 1024; paso = nseg / 2; w = 0.5 - 0.5 * cos(2 * pi * (0:nseg-1)' / (nseg - 1));
P = zeros(nseg / 2 + 1, 1); m = 0;
for i0 = 1:paso:numel(ffi) - nseg + 1
    x = ffi(i0:i0 + nseg - 1); x = (x - mean(x)) .* w;
    X = fft(x); P = P + abs(X(1:nseg / 2 + 1)).^2; m = m + 1;
end
f = (0:nseg / 2)' * fs / nseg;
f3 = figure('Name', 'F3 espectro de finalFF', 'Position', [110 110 700 450]);
if m > 0
    semilogy(f, P / m); xlabel('f (Hz)'); ylabel('densidad (u.a.)'); grid on;
    title(sprintf('Espectro de finalFF (%d segmentos)', m));
    E = cumsum(P); f95 = f(find(E >= 0.95 * E(end), 1));
    fprintf('Espectro: f95 de finalFF = %.1f Hz\n', f95);
end
guardar(f3, dfig, 'F3_espectro');
fprintf('\nVeredicto: %s\n', ternario(c1 && c2 && isequal(c3, true) && isequal(c4, true), ...
    'finalFF parece una señal de FFB utilizable (falta decidir K_FFB y su signo).', ...
    'finalFF NO queda validada con esta corrida; ver qué criterio falla.'));
diary off

function s = pasa(c), if c, s = 'PASA'; else, s = 'NO PASA'; end, end
function s = ternario(c, a, b), if c, s = a; else, s = b; end, end
function guardar(h, d, n)
    exportgraphics(h, fullfile(d, [n '.png']), 'Resolution', 150);
    savefig(h, fullfile(d, [n '.fig']));
end
