%% s21_servo_volante.m
% Revisión del controlador del volante (lazo de posición de Odrive-Wheel en la placa), pedida
% por el profesor el 2026-10-08: "los controladores dan setpoint de ángulo del volante, toca ver
% cómo el volante llega a esos setpoints, tal vez el problema esté ahí".
%
% Cómo está montado (código y configuración, VERIFICADO EN ARCHIVO REAL):
%   - cascada de ODrive v0.5.6 en la placa, 8 kHz, INPUT_MODE_PASSTHROUGH:
%       ω_ref = pos_gain·(θ_ref − θ) (+ vel_ff), saturada a vel_limit
%       τ = input_torque + vel_gain·(ω_ref − ω) + vel_integrator_gain·∫(ω_ref − ω)
%   - ganancias S6: pos_gain 50 (vuelta/s por vuelta), vel_gain 0,8 N·m/(vuelta/s),
%     vel_integrator_gain 2,0 N·m/(vuelta/s)/s; vel_limit 2 vueltas/s; current_lim 10 A;
%     torque_constant configurada 0,55 N·m/A;
%   - la consigna llega por "p 0 <pos>" cada ciclo del controlador lateral (20 Hz) y el hilo
%     espejo la repite cada ~5 ms SIN interpolar ni mandar vel_ff: la placa ve escalones de
%     consigna cada 50 ms.
%
% Qué hace:
%   1. Estima la inercia J, el amortiguamiento viscoso b y la fricción de Coulomb c del volante con
%      los escalones y la prueba de tasa de S1 (τ = 0,55·Iq en las unidades de la placa: las
%      ganancias de la placa están en esas mismas unidades, así que los polos no dependen de que
%      Kt real sea 0,55).
%   2. Polos del lazo de posición con S6 (ζ y ω_n del par dominante).
%   3. Simula la cascada (con vel_limit, límite de corriente y Coulomb) y la valida contra los
%      escalones de 20° y 45° medidos en S1.
%   4. Ruido de medida: desviación de ω e Iq con el volante quieto en reposo (encoder + PLL).
%   5. Propuestas: (a) mismas ganancias con consigna interpolada + vel_ff (orden uno, sin retardo
%      añadido); (b) ganancias con más amortiguamiento. Las compara simulando la reproducción en
%      lazo abierto del MPC del paper 1 (comando real de c02, 20 Hz) y valida el modelo con el
%      seguimiento medido en c02.
% Inferencia: DESCRIPTIVA / modelo. Estado: EXPLORATORIO. Solo MATLAB base.
% Salidas: results/exploratory/servo_volante/s21_*.csv, figures/draft/servo_volante/F1–F4,
%          logs/s21_servo_volante.log
% Uso:  matlab -batch "run('scripts/s21_servo_volante.m')"
%
% Guía de observación para el investigador (AGENTS.md §17)
%   F1  Ajuste de J, b y c: torque medido frente al predicho en los escalones. ¿Nube sobre la
%       diagonal? Si no, el modelo de un cuerpo rígido no basta (holgura, flexión).
%   F2  Escalones de 20° y 45°: medido frente a simulado con S6. ¿El modelo reproduce el
%       sobrepaso y la oscilación de ~2,5 Hz? Si sí, la causa está en las ganancias, no en ruido.
%   F3  Mapa de amortiguamiento ζ del par dominante frente a pos_gain y vel_gain (S6 marcado).
%       ¿Dónde está S6? ¿Qué cambio pequeño lo lleva a ζ ≈ 0,7?
%   F4  Reproducción del MPC (c02): error de seguimiento con escalones de 20 Hz (actual), con
%       consigna interpolada + vel_ff y con ganancias amortiguadas. ¿Cuánto baja el error y el
%       sobrepaso? Esto es lo que el lazo lateral "vería" como actuador.
%   Preguntas: ¿el sobrepaso del servo explica la oscilación de Stanley y MPC en HIL? ¿Basta con
%   interpolar la consigna o hay que cambiar ganancias? ¿El ruido del encoder es importante
%   frente al torque de las correcciones?

clearvars
raiz = fileparts(fileparts(mfilename('fullpath')));
if isempty(raiz), raiz = pwd; end
dres = fullfile(raiz, 'results', 'exploratory', 'servo_volante');
dfig = fullfile(raiz, 'figures', 'draft', 'servo_volante');
for dd = {dres, dfig, fullfile(raiz, 'logs')}, if ~exist(dd{1}, 'dir'), mkdir(dd{1}); end, end
diary(fullfile(raiz, 'logs', 's21_servo_volante.log')); diary on
fprintf('s21_servo_volante  %s\n', datestr(now, 31));

KT = 0.55; SIGNO = -1; TWO_PI = 2 * pi;
S6 = struct('pos_gain', 50, 'vel_gain', 0.8, 'vel_int', 2.0);
VEL_LIM = 2.0 * TWO_PI;            % rad/s
TAU_MAX = 10 * KT;                 % N·m (unidades de la placa)

%% 1. Identificación de J, b, c con S1
cS1 = fullfile(raiz, 'data', 'raw', 'caracterizacion_formal', '20261008_094949_formal_S1');
M = readtable(fullfile(cS1, 'muestras.csv'), 'TextType', 'string');
t = M.t_s; w = deg2rad(M.omega_dps); th = deg2rad(M.meas_deg);
iq = fillmissing(M.iq_a, 'linear');
tau = SIGNO * KT * iq;                         % torque en el volante, unidades de la placa
ws = movmean(w, 5); alfa = gradient(ws) ./ gradient(t);
usar = (M.tipo == "escalones" | M.tipo == "tasa") & abs(alfa) > deg2rad(300) & abs(ws) > deg2rad(5);
X = [alfa(usar), ws(usar), sign(ws(usar))];
p = X \ tau(usar);
J = p(1); b = p(2); c = p(3);
r2 = 1 - sum((tau(usar) - X * p).^2) / sum((tau(usar) - mean(tau(usar))).^2);
fprintf('\n1. Identificación (n = %d muestras de escalones y tasa):\n', nnz(usar));
fprintf('   J = %.4f kg·m², b = %.4f N·m·s/rad, c = %.3f N·m (Coulomb), R² = %.2f\n', J, b, c, r2);
if J <= 0, warning('J no positiva: revisar el signo de Iq'); end
f1 = figure('Name', 'F1 ajuste', 'Position', [40 40 600 520]);
plot(tau(usar), X * p, '.', 'MarkerSize', 4); hold on; grid on;
lim = max(abs(tau(usar))); plot([-lim lim], [-lim lim], 'k--');
xlabel('\tau medido = signo·0,55·Iq (N·m)'); ylabel('\tau = J\alpha + b\omega + c·sign(\omega) (N·m)');
title(sprintf('Ajuste J = %.4f, b = %.3f, c = %.2f, R^2 = %.2f', J, b, c, r2));
guardar(f1, dfig, 'F1_ajuste');

%% 2. Polos con S6
[zS6, wnS6, polos] = amortiguamiento(S6, J, b);
fprintf('\n2. Lazo de posición con S6 (lineal, sin saturaciones):\n');
fprintf('   polos: %s\n', mat2str(polos.', 3));
fprintf('   par dominante: ω_n = %.1f rad/s (%.2f Hz), ζ = %.3f\n', wnS6, wnS6 / TWO_PI, zS6);

%% 3. Validación con los escalones de S1
E = readtable(fullfile(cS1, 'escalones.csv'));
f2 = figure('Name', 'F2 escalones', 'Position', [60 60 1000 420]);
amps = [20 45]; V = table();
for q = 1:2
    subplot(1, 2, q); hold on; grid on;
    sel = find(abs(abs(E.hacia_deg - E.desde_deg) - amps(q)) < 1 & E.desde_deg == 0, 3);
    for i = sel'
        k = t >= E.t_cmd_s(i) - 0.01 & t <= E.t_cmd_s(i) + 0.6;
        plot(t(k) - E.t_cmd_s(i), (M.meas_deg(k) - E.desde_deg(i)) / (E.hacia_deg(i) - E.desde_deg(i)), 'Color', [0.6 0.6 0.6]);
    end
    [ts, ys] = simular(@(tt) amps(q) * (tt >= 0), 0.6, S6, J, b, c, VEL_LIM, TAU_MAX, false);
    plot(ts, ys / amps(q), 'r', 'LineWidth', 1.5);
    title(sprintf('Escalón de %d°: medido (gris), modelo S6 (rojo)', amps(q))); xlabel('t (s)'); ylabel('respuesta / A');
    sp = 100 * (max(ys) / amps(q) - 1);
    V = [V; table(amps(q), sp, 'VariableNames', {'amplitud_deg', 'sobrepaso_modelo_pct'})]; %#ok<AGROW>
end
disp(V); guardar(f2, dfig, 'F2_escalones');

%% 4. Ruido de medida con el volante quieto (reposo inicial de S1)
rep = M.tipo == "reposo" & t < 2.0;
fprintf('\n4. Ruido en reposo (%d muestras): σ(ω) = %.2f °/s, σ(Iq) = %.3f A, σ(θ) = %.3f°\n', ...
    nnz(rep), std(M.omega_dps(rep)), std(iq(rep)), std(M.meas_deg(rep)));
fprintf('   Torque de ruido por el lazo de velocidad con S6: vel_gain·σ(ω) = %.3f N·m\n', ...
    S6.vel_gain / TWO_PI * deg2rad(std(M.omega_dps(rep))));

%% 5. Mapa de amortiguamiento y propuestas
pg = 5:5:80; vg = 0.4:0.2:4.0; Z = nan(numel(vg), numel(pg)); W = Z;
for i = 1:numel(vg)
    for j = 1:numel(pg)
        g = struct('pos_gain', pg(j), 'vel_gain', vg(i), 'vel_int', vg(i) * S6.vel_int / S6.vel_gain);
        [Z(i, j), W(i, j)] = amortiguamiento(g, J, b);
    end
end
f3 = figure('Name', 'F3 mapa de ζ', 'Position', [80 80 760 520]);
contourf(pg, vg, Z, 0:0.1:1, 'ShowText', 'on'); colorbar; hold on;
plot(S6.pos_gain, S6.vel_gain, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
xlabel('pos\_gain (vuelta/s por vuelta)'); ylabel('vel\_gain (N·m por vuelta/s)');
title('\zeta del par dominante (vel\_integrator\_gain proporcional a vel\_gain como en S6)');
guardar(f3, dfig, 'F3_mapa_zeta');
fprintf('\n5. Mapa de ζ guardado (F3). Búsqueda de ganancias por seguimiento simulado (paso 6).\n');

%% 6. Reproducción del MPC (c02, 900°): validación del modelo y comparación de propuestas
dc = dir(fullfile(raiz, 'data', 'raw', 'caracterizacion', '20261008_111817_reproduccion_p00', '*mpc*'));
C = readtable(fullfile(dc(1).folder, dc(1).name, 'odrive_ciclos.csv'));
Sp = readtable(fullfile(dc(1).folder, dc(1).name, 'odrive_espejo.csv'));
tc = C.t_perf_s - C.t_perf_s(1); cmd = C.theta_cmd_deg;
k0 = find(tc > 40, 1); k1 = find(tc > 100, 1);            % 60 s de la vuelta, sin la salida
tc = tc(k0:k1) - tc(k0); cmd = cmd(k0:k1) - cmd(k0);
te = Sp.t_perf_s - C.t_perf_s(1) - (C.t_perf_s(k0) - C.t_perf_s(1));
me = Sp.theta_meas_deg - C.theta_cmd_deg(k0); ke = te >= 0 & te <= tc(end);
zoh = @(tt) interp1(tc, cmd, tt, 'previous', cmd(end));
vff = [0; diff(cmd) ./ diff(tc)];
foh = @(tt) interp1(tc, cmd, tt, 'previous', cmd(end)) + interp1(tc, vff, tt, 'previous', 0) .* (tt - interp1(tc, tc, tt, 'previous', tc(end)));
% Búsqueda: ζ ≥ 0,5, ruido de torque vel_gain·σ(ω) ≤ 0,1 N·m; se elige el menor RMS de seguimiento
% del MPC (30 s, consigna interpolada + vel_ff). vel_integrator_gain en la misma proporción que S6.
sw = deg2rad(std(M.omega_dps(rep)));
B = table();
for pgc = [20 30 40 50 60]
    for vgc = [1.2 1.6 2.4 3.2 4.8 6.4 8.0]
        g = struct('pos_gain', pgc, 'vel_gain', vgc, 'vel_int', vgc * S6.vel_int / S6.vel_gain);
        [zc, wc] = amortiguamiento(g, J, b);
        ruido = vgc / (2 * pi) * sw;
        if zc < 0.5 || ruido > 0.1, continue; end
        [ts, ys] = simular(foh, 30, g, J, b, c, VEL_LIM, TAU_MAX, true, @(tt) interp1(tc, vff, tt, 'previous', 0));
        e = ys - interp1(tc, cmd, ts, 'linear');
        B = [B; table(pgc, vgc, g.vel_int, zc, wc / (2 * pi), ruido, rms(e), max(abs(e)), ...
            'VariableNames', {'pos_gain', 'vel_gain', 'vel_int', 'zeta', 'fn_hz', 'ruido_nm', 'rms_err_deg', 'max_err_deg'})]; %#ok<AGROW>
    end
end
B = sortrows(B, 'rms_err_deg'); disp(B(1:min(8, height(B)), :))
writetable(B, fullfile(dres, 's21_busqueda_ganancias.csv'));
AM = struct('pos_gain', B.pos_gain(1), 'vel_gain', B.vel_gain(1), 'vel_int', B.vel_int(1));
[zAM, wnAM] = amortiguamiento(AM, J, b);
fprintf('Ganancias propuestas: pos_gain %g, vel_gain %.1f, vel_integrator_gain %.1f -> ζ = %.2f, f_n = %.2f Hz\n', ...
    AM.pos_gain, AM.vel_gain, AM.vel_int, zAM, wnAM / (2 * pi));
casos = {'S6, escalones de 20 Hz (actual)', S6, zoh, false; ...
         'S6, consigna interpolada + vel_ff', S6, foh, true; ...
         'Propuestas, escalones de 20 Hz', AM, zoh, false; ...
         'Propuestas, interpolada + vel_ff', AM, foh, true};
f4 = figure('Name', 'F4 reproducción MPC', 'Position', [100 100 1100 460]); hold on; grid on;
plot(te(ke), me(ke), 'Color', [0.5 0.5 0.5], 'DisplayName', 'medido en c02 (S6)');
R = table();
for q = 1:size(casos, 1)
    [ts, ys] = simular(casos{q, 3}, tc(end), casos{q, 2}, J, b, c, VEL_LIM, TAU_MAX, casos{q, 4}, ...
        @(tt) interp1(tc, vff, tt, 'previous', 0));
    ref = interp1(tc, cmd, ts, 'linear');
    err = ys - ref;
    R = [R; table(string(casos{q, 1}), rms(err), prctile(abs(err), 95), max(abs(err)), ...
        'VariableNames', {'caso', 'rms_err_deg', 'p95_err_deg', 'max_err_deg'})]; %#ok<AGROW>
    plot(ts, ys, 'DisplayName', casos{q, 1});
    if q == 1
        ym = interp1(te(ke), me(ke), ts, 'linear', NaN);
        fprintf('\n6. Validación del modelo con c02 (MPC, 60 s): RMS(medido − modelo S6) = %.2f°, RMS(medido − consigna) = %.2f°, RMS(modelo − consigna) = %.2f°\n', ...
            rms(ym - ys, 'omitnan'), rms(ym - ref, 'omitnan'), rms(err));
    end
end
plot(tc, cmd, 'k:', 'DisplayName', 'consigna (20 Hz)');
xlim([10 20]); xlabel('t (s)'); ylabel('\theta (°)'); legend('Location', 'best');
title('Reproducción del MPC: medido y simulado con las propuestas');
guardar(f4, dfig, 'F4_reproduccion_mpc');
disp(R)
writetable(R, fullfile(dres, 's21_propuestas.csv'));
writetable(table(J, b, c, r2, zS6, wnS6, AM.pos_gain, AM.vel_gain, AM.vel_int, zAM, wnAM, ...
    'VariableNames', {'J', 'b', 'c_coulomb', 'R2', 'zeta_S6', 'wn_S6', 'pos_gain_am', 'vel_gain_am', 'vel_int_am', 'zeta_am', 'wn_am'}), ...
    fullfile(dres, 's21_modelo.csv'));
diary off

%% ---------------------------------------------------------------------------------------
function [zeta, wn, p] = amortiguamiento(g, J, b)
    % Cascada lineal en rad: J s^3 + (Kv+b) s^2 + (Kv·Kp + Ki) s + Ki·Kp = 0
    Kp = g.pos_gain; Kv = g.vel_gain / (2 * pi); Ki = g.vel_int / (2 * pi);
    p = roots([J, Kv + b, Kv * Kp + Ki, Ki * Kp]);
    cpx = p(abs(imag(p)) > 1e-6);
    if isempty(cpx), [~, i] = min(abs(p)); zeta = 1; wn = abs(p(i)); return; end
    [~, i] = min(-real(cpx) ./ abs(cpx));          % el par menos amortiguado
    wn = abs(cpx(i)); zeta = -real(cpx(i)) / wn;
end

function [ts, ys] = simular(ref, T, g, J, b, c, vlim, tmax, conVff, vffFun)
    % Cascada de ODrive a 8 kHz con vel_limit, límite de torque y Coulomb (con zona de pegue).
    if nargin < 10, vffFun = @(tt) 0 * tt; end
    dt = 1 / 8000; n = round(T / dt); ts = (0:n - 1)' * dt; ys = zeros(n, 1);
    th = 0; w = 0; integ = 0;
    Kp = g.pos_gain; Kv = g.vel_gain / (2 * pi); Ki = g.vel_int / (2 * pi);
    rr = deg2rad(ref(ts)); vf = zeros(n, 1);
    if conVff, vf = deg2rad(vffFun(ts)); end
    for k = 1:n
        wref = Kp * (rr(k) - th) + vf(k);
        wref = max(-vlim, min(vlim, wref));
        e = wref - w;
        tau = Kv * e + integ;
        tau = max(-tmax, min(tmax, tau));
        if abs(tau) < tmax, integ = integ + Ki * e * dt; end
        if abs(w) < 1e-3 && abs(tau - b * w) <= c
            w = 0;                                  % pegado
        else
            a = (tau - b * w - c * sign(w + 1e-9 * sign(tau))) / J;
            w = w + a * dt;
        end
        th = th + w * dt;
        ys(k) = rad2deg(th);
    end
    ys = ys(1:20:end); ts = ts(1:20:end);          % 400 Hz para graficar
end

function guardar(h, d, n)
    exportgraphics(h, fullfile(d, [n '.png']), 'Resolution', 150);
    savefig(h, fullfile(d, [n '.fig']));
end
