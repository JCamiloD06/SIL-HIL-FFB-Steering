%% simulacion_lazo_cerrado.m
% Closed-loop lateral simulation of vehicle and steering actuator dynamics.
%
% Author: Juan Camilo Diaz-Lopez
% Universidad Nacional de Colombia, Sede La Paz
% Research Paper: Software-to-Hardware-in-the-Loop Transferability of
%                 Autonomous Path Tracking under Physical Steering Dynamics
%                 and Force-Feedback Loading (Ingenieria e Investigacion)
%
% This script simulates the lateral tracking behavior of geometric controllers
% (Stanley, Pure Pursuit) coupled with the physical steering actuator model,
% demonstrating delay-induced limit cycles, rate saturation, and lateral error.

clearvars; close all; clc;

%% 1. Parameters Setup
V = vehiculo(1390, 0.4, 90e3, 0.92);

P = struct('k_st', 2.037435, 'eps', 1.0, 'L0', 2.182824, 'kv', 0.475382, ...
    'dmax', 0.403, 'tasa', 0.5235987755982988, 'Ts', 0.05, 'kw', 19.48878817, ...
    'gAC', 0.485 / 0.403, 'tauAC', 0.03, 'Y0', 0.5, 'J', 0.0189, 'c', 0.176, ...
    'vlim', 4 * pi, 'tmax', 5.5);

S6 = [50 0.8 2.0]; 
PROP = [20 3.2 8.0];

conds = {'SIL', [], false; ...
         'S6 escalones', S6, false; ...
         'S6 interpolada', S6, true; ...
         'Propuesta escalones', PROP, false; ...
         'Propuesta interpolada', PROP, true};

vel = [60 80 100 120 130 140 150];
ctrls = {'stanley', 'pure_pursuit'};

%% 2. Execution across Conditions and Speeds
fprintf('=================================================================\n');
fprintf('Running Closed-Loop Lateral Tracking Simulations...\n');
fprintf('=================================================================\n');

M = table();
for ic = 1:2
    for iv = 1:numel(vel)
        for k = 1:size(conds, 1)
            r = simular_lazo(ctrls{ic}, vel(iv), conds{k, 2}, conds{k, 3}, V, P, 20);
            M = [M; table(string(ctrls{ic}), vel(iv), string(conds{k, 1}), ...
                r.osc_final, r.ey_final, r.estable, r.f_osc, ...
                'VariableNames', {'controlador', 'v_kmh', 'condicion', 'osc_delta_deg', 'rms_ey_m', 'estable', 'f_osc_hz'})]; %#ok<AGROW>
        end
    end
end

disp(M(1:15, :));
fprintf('... and %d more simulation rows.\n\n', height(M) - 15);

%% 3. Interactive Visualization
figure('Name', 'Closed-Loop Tracking Map', 'Color', 'w', 'Position', [100 100 1100 450]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact');

for ic = 1:2
    nexttile; hold on; grid on; box on;
    for k = 1:size(conds, 1)
        q = M.controlador == ctrls{ic} & M.condicion == conds{k, 1};
        semilogy(M.v_kmh(q), max(M.osc_delta_deg(q), 1e-3), '-o', 'LineWidth', 1.2, 'DisplayName', conds{k, 1});
    end
    set(gca, 'YScale', 'log'); 
    xlabel('Longitudinal Speed v (km/h)'); 
    ylabel('RMS of \delta in last 5 s [deg]');
    title(strrep(ctrls{ic}, '_', ' ')); 
    legend('Location', 'northwest');
end

%% Local Functions
function V = vehiculo(m, mu, Cr, eta)
    L = 2.634; a = 1.054; b = L - a;
    Fzf = m * 9.81 * (b / L); Fzr = m * 9.81 * (a / L);
    Cf = (Fzf / Fzr) * Cr;
    Iz = (m / 1390) * 2200;
    V = struct('m', m, 'Iz', Iz, 'a', a, 'b', b, 'L', L, ...
        'Cf', Cf, 'Cr', Cr, 'mu', mu, 'eta', eta, ...
        'Fzf', Fzf, 'Fzr', Fzr, 'Ff_max', mu * Fzf, 'Fr_max', mu * Fzr);
end

function r = simular_lazo(ctrl, v_kmh, cond_act, interp, V, P, T)
    dt = 0.001; N = round(T / dt); U = v_kmh / 3.6;
    K_us = (V.m / V.L) * (V.b / (2 * V.Cf) - V.a / (2 * V.Cr));
    if strcmp(ctrl, 'stanley')
        k_st = P.k_st; eps_st = P.eps;
    else
        L0 = P.L0; kv = P.kv;
    end
    kw = P.kw;
    if isempty(cond_act)
        tiene_act = false;
    else
        tiene_act = true;
        retardo_s = cond_act(1) * 1e-3;
        bw_hz = cond_act(2);
        rate_hw = cond_act(3) * 180 / pi;
        w_bw = 2 * pi * bw_hz;
        buf_len = max(1, round(retardo_s / dt));
        buf = zeros(buf_len, 1);
        buf_ptr = 1;
        filtro_th = 0;
        th_act = 0;
        sat_tasa_hw = rate_hw;
    end
    paso_ctrl = round(P.Ts / dt);
    u_held = 0;
    vy = 0; w = 0; psi = 0; Y = P.Y0;
    ir = 0;
    N_reg = round(T / P.Ts) + 5;
    r = struct('t', zeros(N_reg, 1), 'delta_rueda', zeros(N_reg, 1), ...
               'theta', zeros(N_reg, 1), 'ey', zeros(N_reg, 1));
    for i = 1:N
        t = (i - 1) * dt;
        if mod(i - 1, paso_ctrl) == 0
            if strcmp(ctrl, 'stanley')
                ey = Y; epsi = psi;
                u_cmd = -epsi - atan2(k_st * ey, U + eps_st);
            else
                Ld = max(2.0, L0 + kv * U);
                delta_x = Ld * cos(psi);
                delta_y = Y + Ld * sin(psi);
                Ld_real = hypot(delta_x, delta_y);
                alpha = -atan2(delta_y, delta_x) - psi;
                u_cmd = atan2(2 * V.L * sin(alpha), Ld_real);
            end
            u_held = max(-P.dmax, min(P.dmax, u_cmd));
        end
        if ~tiene_act
            dr = u_held;
            r_th = dr * kw * 180 / pi;
        else
            th_target = u_held * kw * 180 / pi;
            buf(buf_ptr) = th_target;
            buf_ptr = mod(buf_ptr, buf_len) + 1;
            th_del = buf(buf_ptr);
            d_filtro = w_bw * (th_del - filtro_th);
            filtro_th = filtro_th + d_filtro * dt;
            if interp
                d_pos = filtro_th - th_act;
                d_pos = max(-sat_tasa_hw * dt, min(sat_tasa_hw * dt, d_pos));
                th_act = th_act + d_pos;
            else
                th_act = filtro_th;
            end
            r_th = th_act;
            dr = (th_act / kw) * (pi / 180);
        end
        af = (vy + V.a * w) / U - dr; 
        ar = (vy - V.b * w) / U;
        Ff = -V.Cf * af; 
        Fr = -V.Cr * ar;
        vy = vy + ((Ff + Fr) / V.m - U * w) * dt;
        w = w + ((V.a * Ff - V.b * Fr) / V.Iz) * dt;
        psi = psi + w * dt;
        Y = Y + (U * sin(psi) + vy * cos(psi)) * dt;
        if abs(Y) > 50, break; end
        if mod(i, 20) == 0
            ir = ir + 1; 
            r.t(ir) = t; 
            r.delta_rueda(ir) = dr; 
            r.theta(ir) = r_th; 
            r.ey(ir) = Y;
        end
    end
    r.t = r.t(1:ir); r.delta_rueda = r.delta_rueda(1:ir); r.theta = r.theta(1:ir); r.ey = r.ey(1:ir);
    ult = r.t > r.t(end) - 5;
    r.osc_final = rms(rad2deg(r.delta_rueda(ult)));
    r.ey_final = rms(r.ey(ult));
    r.estable = r.t(end) > T - 0.1 && r.ey_final < 0.05;
    x = r.delta_rueda(ult) - mean(r.delta_rueda(ult));
    cz = find(x(1:end - 1) .* x(2:end) < 0);
    r.f_osc = NaN; 
    ii = find(ult);
    if numel(cz) > 3
        r.f_osc = (numel(cz) - 1) / (2 * (r.t(ii(cz(end))) - r.t(ii(cz(1)))));
    end
end
