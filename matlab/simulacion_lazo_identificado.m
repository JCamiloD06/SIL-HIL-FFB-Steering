%% simulacion_lazo_identificado.m
% Closed-loop simulation with identified actuator dynamics across vehicle speeds.
% Author: Juan Camilo Diaz-Lopez
% Department of Mechanical and Mechatronics Engineering
% Universidad Nacional de Colombia, Sede La Paz
% Research Paper: Physical Steering Dynamics and Force-Feedback Loading
%                 in Autonomous Path Tracking (Ingenieria e Investigacion)
%
clearvars
raiz = fileparts(fileparts(mfilename('fullpath')));
if isempty(raiz), raiz = pwd; end
dres = fullfile(raiz, 'results', 'exploratory', 'simulacion_lazo');
dfig = fullfile(raiz, 'figures', 'draft', 'simulacion_lazo');
for dd = {dres, dfig, fullfile(raiz, 'logs')}, if ~exist(dd{1}, 'dir'), mkdir(dd{1}); end, end
diary(fullfile(raiz, 'logs', 's25_simulacion_lazo_identificado.log')); diary on
fprintf('s25_simulacion_lazo_identificado  %s\n', datestr(now, 31));
load(fullfile(raiz, 'results', 'exploratory', 'vehiculo_identificado', 's24_modelos.mat'), 'modelos');

P = struct('k_st', 2.037435, 'eps', 1.0, 'L0', 2.182824, 'kv', 0.475382, 'L', 2.634, 'a', 1.054, 'b', 1.58, ...
    'dmax', 0.403, 'tasa', 0.5235987755982988, 'Ts', 0.05, 'kw', 19.48878817, ...
    'J', 0.0189, 'c', 0.176, 'vlim', 4 * pi, 'tmax', 5.5, 'Y0', 0.05);
S6 = [50 0.8 2.0]; PROP = [20 3.2 8.0];
conds = {'SIL', [], false; 'S6 escalones', S6, false; 'S6 interpolada', S6, true; ...
         'Propuesta escalones', PROP, false; 'Propuesta interpolada', PROP, true};
vel = [66 92 123 144];                                     % velocidades medias de las bandas de s24
ctrls = {'stanley', 'pure_pursuit'};
M = table();
for ic = 1:2
    for iv = 1:numel(vel)
        mod_ = modelos(iv);
        for k = 1:size(conds, 1)
            r = simular(ctrls{ic}, vel(iv), conds{k, 2}, conds{k, 3}, mod_, P, 20);
            M = [M; table(string(ctrls{ic}), vel(iv), string(conds{k, 1}), r.osc, r.ey, r.f, ...
                'VariableNames', {'controlador', 'v_kmh', 'condicion', 'osc_delta_deg', 'rms_ey_m', 'f_osc_hz'})]; %#ok<AGROW>
        end
    end
end
disp(M); writetable(M, fullfile(dres, 's25_mapa.csv'));

% Búsqueda de ganancias del servo con consigna interpolada: Stanley a 144 y 123 km/h y MPC no se
% simula (QP). Ruido de torque acotado por vel_gain ≤ 8 (s21: σ(ω) = 5,3 °/s → ≤ 0,12 N·m).
G = table();
for pg = [10 20 30 40 50]
    for vg = [0.8 1.6 3.2 4.8 6.4 8.0]
        g = [pg vg vg * 2.5];
        o144 = simular('stanley', 144, g, true, modelos(4), P, 20);
        o123 = simular('stanley', 123, g, true, modelos(3), P, 20);
        opp = simular('pure_pursuit', 144, g, true, modelos(4), P, 20);
        G = [G; table(pg, vg, vg * 2.5, o144.osc, o144.ey, o123.osc, opp.osc, ...
            'VariableNames', {'pos_gain', 'vel_gain', 'vel_int', 'stanley144_osc', 'stanley144_ey', 'stanley123_osc', 'pp144_osc'})]; %#ok<AGROW>
    end
end
G = sortrows(G, 'stanley144_osc'); fprintf('\nBúsqueda de ganancias (consigna interpolada):\n'); disp(G(1:10, :))
writetable(G, fullfile(dres, 's25_busqueda_ganancias.csv'));

f1 = figure('Name', 'F1', 'Position', [40 40 1100 430]); tl = tiledlayout(1, 2, 'TileSpacing', 'compact');
for ic = 1:2
    nexttile; hold on; grid on;
    for k = 1:size(conds, 1)
        q = M.controlador == ctrls{ic} & M.condicion == conds{k, 1};
        plot(M.v_kmh(q), max(M.osc_delta_deg(q), 1e-4), '-o', 'DisplayName', conds{k, 1});
    end
    set(gca, 'YScale', 'log'); xlabel('v (km/h)'); ylabel('RMS de \delta, últimos 5 s (°)');
    title(strrep(ctrls{ic}, '_', '\_')); legend('Location', 'northwest');
end
title(tl, 'Lazo con el vehículo identificado en AC: oscilación remanente tras e_y = 0,05 m');
exportgraphics(f1, fullfile(dfig, 's25_F1_mapa.png'), 'Resolution', 150); savefig(f1, fullfile(dfig, 's25_F1_mapa.fig'));
f2 = figure('Name', 'F2', 'Position', [60 60 1100 600]); tl = tiledlayout(2, 1, 'TileSpacing', 'compact');
for k = [1 2 5]
    r = simular('stanley', 144, conds{k, 2}, conds{k, 3}, modelos(4), P, 20);
    nexttile(1); hold on; grid on; plot(r.t, rad2deg(r.d), 'DisplayName', conds{k, 1}); ylabel('\delta (°)');
    nexttile(2); hold on; grid on; plot(r.t, r.Y, 'DisplayName', conds{k, 1}); ylabel('e_y (m)'); xlabel('t (s)');
end
nexttile(1); legend('Location', 'best'); title(tl, 'Stanley a 144 km/h con el vehículo identificado');
exportgraphics(f2, fullfile(dfig, 's25_F2_stanley_144.png'), 'Resolution', 150); savefig(f2, fullfile(dfig, 's25_F2_stanley_144.fig'));
diary off

function r = simular(ctrl, v_kmh, gan, interp, mod_, P, T)
    dt = 1 / 2000; n = round(T / dt); U = v_kmh / 3.6;
    sr = c2d(ss(mod_.r), dt); sa = c2d(ss(mod_.ay), dt);
    xr = zeros(size(sr.A, 1), 1); xa = zeros(size(sa.A, 1), 1);
    psi = 0; Y = P.Y0; Yd = 0;
    th = 0; om = 0; integ = 0; dcmd = 0; dprev = 0; thk = 0; thk_1 = 0; tk = 0; vk = 0;
    sil = isempty(gan); if ~sil, Kp = gan(1); Kv = gan(2) / (2 * pi); Ki = gan(3) / (2 * pi); end
    paso_c = round(P.Ts / dt); nreg = floor(n / 20); r.t = zeros(nreg, 1); r.d = r.t; r.Y = r.t; ir = 0;
    for i = 1:n
        t = (i - 1) * dt;
        if mod(i - 1, paso_c) == 0
            switch ctrl
                case 'stanley'
                    d = -psi - atan(P.k_st * (Y + P.a * sin(psi)) / (U + P.eps));
                case 'pure_pursuit'
                    Ld = P.L0 + P.kv * U; Yr = Y - P.b * sin(psi);
                    d = atan2(2 * P.L * sin(atan2(-Yr, Ld) - psi), Ld);
            end
            d = max(-P.dmax, min(P.dmax, d));
            d = dprev + max(-P.tasa * P.Ts, min(P.tasa * P.Ts, d - dprev)); dprev = d; dcmd = d;
            thk_1 = thk; thk = P.kw * dcmd; vk = (thk - thk_1) / P.Ts; tk = t;
        end
        if sil
            du = dcmd;
        else
            if interp, ref = thk + vk * min(t - tk, 2 * P.Ts); vff = vk; else, ref = thk; vff = 0; end
            wref = max(-P.vlim, min(P.vlim, Kp * (ref - th) + vff));
            e = wref - om; tau = max(-P.tmax, min(P.tmax, Kv * e + integ));
            if abs(tau) < P.tmax, integ = integ + Ki * e * dt; end
            if abs(om) < 1e-3 && abs(tau) <= P.c, om = 0;
            else, om = om + (tau - P.c * sign(om + 1e-9 * sign(tau))) / P.J * dt; end
            th = th + om * dt; du = th / P.kw;
        end
        rr = sr.C * xr + sr.D * du; xr = sr.A * xr + sr.B * du;
        ay = sa.C * xa + sa.D * du; xa = sa.A * xa + sa.B * du;
        psi = psi + rr * dt; Yd = Yd + ay * dt; Y = Y + Yd * dt;
        if abs(Y) > 50, break; end
        if mod(i, 20) == 0, ir = ir + 1; r.t(ir) = t; r.d(ir) = du; r.Y(ir) = Y; end
    end
    r.t = r.t(1:ir); r.d = r.d(1:ir); r.Y = r.Y(1:ir);
    u = r.t > r.t(end) - 5; r.osc = rms(rad2deg(r.d(u))); r.ey = rms(r.Y(u));
    x = r.d(u) - mean(r.d(u)); ii = find(u); cz = find(x(1:end - 1) .* x(2:end) < 0); r.f = NaN;
    if numel(cz) > 3, r.f = (numel(cz) - 1) / (2 * (r.t(ii(cz(end))) - r.t(ii(cz(1))))); end
end
