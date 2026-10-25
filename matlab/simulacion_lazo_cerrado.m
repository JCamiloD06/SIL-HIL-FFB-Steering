%% simulacion_lazo_cerrado.m
% Closed-loop lateral simulation of vehicle and steering actuator dynamics.
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
diary(fullfile(raiz, 'logs', 's23_simulacion_lazo.log')); diary on
fprintf('s23_simulacion_lazo  %s\n', datestr(now, 31));

V = vehiculo(1390, 0.4, 90e3, 0.92);
P = struct('k_st', 2.037435, 'eps', 1.0, 'L0', 2.182824, 'kv', 0.475382, ...
    'dmax', 0.403, 'tasa', 0.5235987755982988, 'Ts', 0.05, 'kw', 19.48878817, ...
    'gAC', 0.485 / 0.403, 'tauAC', 0.03, 'Y0', 0.5, 'J', 0.0189, 'c', 0.176, 'vlim', 4 * pi, 'tmax', 5.5);
S6 = [50 0.8 2.0]; PROP = [20 3.2 8.0];
conds = {'SIL', [], false; 'S6 escalones', S6, false; 'S6 interpolada', S6, true; ...
         'Propuesta escalones', PROP, false; 'Propuesta interpolada', PROP, true};
vel = [60 80 100 120 130 140 150];
ctrls = {'stanley', 'pure_pursuit'};
M = table();
for ic = 1:2
    for iv = 1:numel(vel)
        for k = 1:size(conds, 1)
            r = simular_lazo(ctrls{ic}, vel(iv), conds{k, 2}, conds{k, 3}, V, P, 20);
            M = [M; table(string(ctrls{ic}), vel(iv), string(conds{k, 1}), r.osc_final, r.ey_final, r.estable, r.f_osc, ...
                'VariableNames', {'controlador', 'v_kmh', 'condicion', 'osc_delta_deg', 'rms_ey_m', 'estable', 'f_osc_hz'})]; %#ok<AGROW>
        end
    end
end
disp(M)
writetable(M, fullfile(dres, 's23_mapa.csv'));

% F1: mapa
f1 = figure('Name', 'F1 mapa', 'Position', [40 40 1100 430]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact');
for ic = 1:2
    nexttile; hold on; grid on;
    for k = 1:size(conds, 1)
        q = M.controlador == ctrls{ic} & M.condicion == conds{k, 1};
        semilogy(M.v_kmh(q), max(M.osc_delta_deg(q), 1e-3), '-o', 'DisplayName', conds{k, 1});
    end
    set(gca, 'YScale', 'log'); xlabel('v (km/h)'); ylabel('RMS de \delta en los últimos 5 s (°)');
    title(strrep(ctrls{ic}, '_', '\_')); legend('Location', 'northwest');
end
title(tl, 'Simulación del lazo: oscilación que queda tras una perturbación de 0,5 m');
guardar(f1, dfig, 'F1_mapa');

% F2: trazas a 140 km/h, Stanley
f2 = figure('Name', 'F2 trazas', 'Position', [60 60 1100 650]);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact');
for k = [1 2 5]
    r = simular_lazo('stanley', 140, conds{k, 2}, conds{k, 3}, V, P, 20);
    nexttile(1); hold on; plot(r.t, rad2deg(r.delta_rueda), 'DisplayName', conds{k, 1}); ylabel('\delta rueda (°)'); grid on;
    nexttile(2); hold on; plot(r.t, r.theta, 'DisplayName', conds{k, 1}); ylabel('\theta volante (°)'); grid on;
    nexttile(3); hold on; plot(r.t, r.ey, 'DisplayName', conds{k, 1}); ylabel('e_y (m)'); xlabel('t (s)'); grid on;
end
nexttile(1); legend('Location', 'best');
title(tl, 'Stanley a 140 km/h: SIL, S6 con escalones (hoy) y servo propuesto');
guardar(f2, dfig, 'F2_trazas_stanley_140');

% Sensibilidad a los supuestos del vehículo (Stanley a 140 km/h)
fprintf('\nSensibilidad (Stanley, 140 km/h): osc. final de δ (°) en SIL / S6 escalones / propuesta interpolada\n');
for caso = {{'base', 1390, 0.4, 90e3, 0.92, 0.03}, {'masa +20 %', 1670, 0.4, 90e3, 0.92, 0.03}, ...
            {'Cr −30 %', 1390, 0.4, 63e3, 0.92, 0.03}, {'subviraje 2 °/g', 1390, 0.4, 90e3, 2.0, 0.03}, ...
            {'τ_AC 0,06 s', 1390, 0.4, 90e3, 0.92, 0.06}, {'reparto 50/50', 1390, 0.5, 90e3, 0.92, 0.03}}
    cc = caso{1}; Vc = vehiculo(cc{2}, cc{3}, cc{4}, cc{5}); Pc = P; Pc.tauAC = cc{6};
    o = zeros(1, 3); ks = [1 2 5];
    for j = 1:3, r = simular_lazo('stanley', 140, conds{ks(j), 2}, conds{ks(j), 3}, Vc, Pc, 20); o(j) = r.osc_final; end
    fprintf('   %-16s %8.3f  %8.3f  %8.3f\n', cc{1}, o);
end
diary off

%% ---------------------------------------------------------------------------------------
function V = vehiculo(m, frac_a, Cr, Kus_deg_g)
    L = 2.634; a = frac_a * L; b = L - a;
    Kus = deg2rad(Kus_deg_g) / 9.81;                          % rad por m/s²
    Cf = b / (Kus * L / m + a / Cr);
    V = struct('m', m, 'L', L, 'a', a, 'b', b, 'Iz', m * a * b, 'Cf', Cf, 'Cr', Cr);
end

function r = simular_lazo(ctrl, v_kmh, gan, interp, V, P, T)
    dt = 1 / 2000; n = round(T / dt); U = v_kmh / 3.6;
    Y = P.Y0; psi = 0; vy = 0; w = 0;                         % vehículo
    th = 0; om = 0; integ = 0;                                % servo (rad)
    dcmd = 0; dprev = 0; thk = 0; thk_1 = 0; tk = 0; vk = 0;  % controlador
    nret = round(P.tauAC / dt);
    buf_d = zeros(nret + 1, 1); buf_s = zeros(nret + 1, 3);   % retardos de actuación y de lectura
    paso_c = round(P.Ts / dt);
    sil = isempty(gan);
    if ~sil, Kp = gan(1); Kv = gan(2) / (2 * pi); Ki = gan(3) / (2 * pi); end
    nreg = floor(n / 20); r.t = zeros(nreg, 1); r.delta_rueda = r.t; r.theta = r.t; r.ey = r.t; ir = 0;
    for i = 1:n
        t = (i - 1) * dt;
        % lectura retardada del estado (como AC → memoria compartida)
        buf_s = [buf_s(2:end, :); Y, psi, vy];
        Ys = buf_s(1, 1); ps = buf_s(1, 2);
        if mod(i - 1, paso_c) == 0
            switch ctrl
                case 'stanley'
                    eyf = Ys + V.a * sin(ps);
                    d = -ps - atan(P.k_st * eyf / (U + P.eps));
                case 'pure_pursuit'
                    Ld = P.L0 + P.kv * U; Yr = Ys - V.b * sin(ps);
                    alfa = atan2(-Yr, Ld) - ps;
                    d = atan2(2 * V.L * sin(alfa), Ld);
            end
            d = max(-P.dmax, min(P.dmax, d));
            d = dprev + max(-P.tasa * P.Ts, min(P.tasa * P.Ts, d - dprev)); dprev = d;
            dcmd = d;
            thk_1 = thk; thk = P.kw * dcmd; vk = (thk - thk_1) / P.Ts; tk = t;
        end
        % actuador
        if sil
            delta_vjoy = dcmd;                                 % δ que TrialSteer pone en vJoy
            r_th = rad2deg(P.kw * dcmd);
        else
            if interp
                ref = thk + vk * min(t - tk, 2 * P.Ts); vff = vk;
            else
                ref = thk; vff = 0;
            end
            wref = max(-P.vlim, min(P.vlim, Kp * (ref - th) + vff));
            e = wref - om; tau = Kv * e + integ;
            tau = max(-P.tmax, min(P.tmax, tau));
            if abs(tau) < P.tmax, integ = integ + Ki * e * dt; end
            if abs(om) < 1e-3 && abs(tau) <= P.c
                om = 0;
            else
                om = om + (tau - P.c * sign(om + 1e-9 * sign(tau))) / P.J * dt;
            end
            th = th + om * dt;
            delta_vjoy = th / P.kw; r_th = rad2deg(th);
        end
        buf_d = [buf_d(2:end); delta_vjoy];
        dr = P.gAC * buf_d(1);                                 % ángulo de rueda en AC
        % vehículo: bicicleta lineal
        af = (vy + V.a * w) / U - dr; ar = (vy - V.b * w) / U;
        Ff = -V.Cf * af; Fr = -V.Cr * ar;
        vy = vy + ((Ff + Fr) / V.m - U * w) * dt;
        w = w + ((V.a * Ff - V.b * Fr) / V.Iz) * dt;
        psi = psi + w * dt;
        Y = Y + (U * sin(psi) + vy * cos(psi)) * dt;
        if abs(Y) > 50, break; end                             % se salió
        if mod(i, 20) == 0
            ir = ir + 1; r.t(ir) = t; r.delta_rueda(ir) = dr; r.theta(ir) = r_th; r.ey(ir) = Y;
        end
    end
    r.t = r.t(1:ir); r.delta_rueda = r.delta_rueda(1:ir); r.theta = r.theta(1:ir); r.ey = r.ey(1:ir);
    ult = r.t > r.t(end) - 5;
    r.osc_final = rms(rad2deg(r.delta_rueda(ult)));
    r.ey_final = rms(r.ey(ult));
    r.estable = r.t(end) > T - 0.1 && r.ey_final < 0.05;
    % frecuencia dominante de δ en los últimos 5 s (cruces por cero)
    x = r.delta_rueda(ult) - mean(r.delta_rueda(ult));
    cz = find(x(1:end - 1) .* x(2:end) < 0);
    r.f_osc = NaN; if numel(cz) > 3, r.f_osc = (numel(cz) - 1) / 2 / (r.t(cz(end)) - r.t(cz(1))) * 1; end
    if numel(cz) > 3, r.f_osc = (numel(cz) - 1) / (2 * (r.t(cz(end)) - r.t(cz(1))) / 1) * 1; end
    ii = find(ult);
    if numel(cz) > 3, r.f_osc = (numel(cz) - 1) / (2 * (r.t(ii(cz(end))) - r.t(ii(cz(1))))); end
end

function guardar(h, d, n)
    exportgraphics(h, fullfile(d, [n '.png']), 'Resolution', 150);
    savefig(h, fullfile(d, [n '.fig']));
end
