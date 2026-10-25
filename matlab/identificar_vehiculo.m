%% identificar_vehiculo.m
% Validation and parameter fitting of linear lateral bicycle dynamics.
% Author: Juan Camilo Diaz-Lopez
% Department of Mechanical and Mechatronics Engineering
% Universidad Nacional de Colombia, Sede La Paz
% Research Paper: Physical Steering Dynamics and Force-Feedback Loading
%                 in Autonomous Path Tracking (Ingenieria e Investigacion)
%
clearvars
raiz = fileparts(fileparts(mfilename('fullpath')));
if isempty(raiz), raiz = pwd; end
dres = fullfile(raiz, 'results', 'exploratory', 'vehiculo_identificado');
dfig = fullfile(raiz, 'figures', 'draft', 'vehiculo_identificado');
for dd = {dres, dfig, fullfile(raiz, 'logs')}, if ~exist(dd{1}, 'dir'), mkdir(dd{1}); end, end
diary(fullfile(raiz, 'logs', 's24_identificar_vehiculo.log')); diary on
fprintf('s24_identificar_vehiculo  %s\n', datestr(now, 31));
TS = 0.05; BANDAS = [50 80; 80 110; 110 135; 135 160]; MIN_S = 4;

est = dir(fullfile(raiz, 'data', 'raw', 'p00_referencia', 'campana_*', 'telemetria.csv'));
val = dir(fullfile(raiz, 'data', 'raw', 'trialsteer', 'SIL_*', 'telemetria.csv'));
signo_ay = NaN;
modelos = struct('banda', {}, 'r', {}, 'ay', {}, 'v_media', {});
A = table();
f1 = figure('Name', 'F1 r/δ', 'Position', [60 60 800 520]); hold on; grid on;
for ib = 1:size(BANDAS, 1)
    [De, signo_ay, ve] = juntar(est, BANDAS(ib, :), TS, MIN_S, signo_ay);
    [Dv, ~, ~] = juntar(val, BANDAS(ib, :), TS, MIN_S, signo_ay);
    if isempty(De), continue; end
    opt = tfestOptions('InitializeMethod', 'iv');
    mr = tfest(De(:, 1, 1), 2, 1, NaN, opt);               % r/δ con retardo libre
    ma = tfest(De(:, 2, 1), 2, 2, NaN, opt);               % a_y/δ
    [~, fe_r] = compare(De(:, 1, 1), mr); [~, fe_a] = compare(De(:, 2, 1), ma);
    fv_r = NaN; fv_a = NaN;
    if ~isempty(Dv), [~, fv_r] = compare(Dv(:, 1, 1), mr); [~, fv_a] = compare(Dv(:, 2, 1), ma); end
    fe_r = media(fe_r); fe_a = media(fe_a); fv_r = media(fv_r); fv_a = media(fv_a);
    modelos(end + 1) = struct('banda', BANDAS(ib, :), 'r', mr, 'ay', ma, 'v_media', ve); %#ok<SAGROW>
    A = [A; table(BANDAS(ib, 1), BANDAS(ib, 2), ve, size(De.y, 2) * 0 + numel(De.ExperimentName), ...
        mr.IODelay, dcgain(mr), fe_r, fv_r, fe_a, fv_a, ...
        'VariableNames', {'v_min', 'v_max', 'v_media_kmh', 'n_tramos', 'retardo_s', 'ganancia_r', ...
        'ajuste_r_est', 'ajuste_r_val', 'ajuste_ay_est', 'ajuste_ay_val'})]; %#ok<AGROW>
    [mag, ~, w] = bode(mr, logspace(-1, 1.5, 200)); semilogx(w / (2 * pi), 20 * log10(squeeze(mag)), ...
        'DisplayName', sprintf('%d–%d km/h', BANDAS(ib, :)));
end
set(gca, 'XScale', 'log'); xlabel('f (Hz)'); ylabel('|r/\delta| (dB, (rad/s)/rad)'); legend('Location', 'best');
title('Guiñada / ángulo de rueda identificada por banda de velocidad (SIL, AC)');
disp(A)
fprintf('Signo usado para a_y = %+d·9,81·acc_g_x (para que a_y y U·r tengan el mismo signo)\n', signo_ay);
writetable(A, fullfile(dres, 's24_ajuste.csv'));
save(fullfile(dres, 's24_modelos.mat'), 'modelos', 'signo_ay');
exportgraphics(f1, fullfile(dfig, 'F1_r_delta.png'), 'Resolution', 150); savefig(f1, fullfile(dfig, 'F1_r_delta.fig'));
diary off

function [D, signo_ay, vmed] = juntar(lista, banda, TS, MIN_S, signo_ay)
    D = []; vv = [];
    for i = 1:numel(lista)
        T = readtable(fullfile(lista(i).folder, lista(i).name), 'TextType', 'string');
        if ~any(T.fase_vuelta == "medida"), continue; end
        r = gradient(unwrap(T.psi_rad)) ./ gradient(T.t_s);
        if isnan(signo_ay)
            q = T.v_kmh > 60; c = corrcoef(9.81 * T.acc_g_x(q), T.v_kmh(q) / 3.6 .* r(q));
            signo_ay = sign(c(1, 2));
        end
        ay = signo_ay * 9.81 * T.acc_g_x;
        en = T.v_kmh >= banda(1) & T.v_kmh < banda(2) & T.fase_vuelta == "medida";
        d = diff([0; en; 0]); ini = find(d == 1); fin = find(d == -1) - 1;
        for j = 1:numel(ini)
            k = ini(j):fin(j);
            if numel(k) * TS < MIN_S, continue; end
            u = T.delta_aplicado_rad(k); u = u - mean(u);
            y = [r(k) - mean(r(k)), ay(k) - mean(ay(k))];
            dj = iddata(y, u, TS, 'OutputName', {'r', 'ay'}, 'InputName', 'delta');
            if isempty(D), D = dj; else, D = merge(D, dj); end
            vv(end + 1) = mean(T.v_kmh(k)); %#ok<AGROW>
        end
    end
    vmed = mean(vv);
end

function m = media(x)
    if iscell(x), x = cellfun(@(z) z(:)', x, 'UniformOutput', false); x = [x{:}]; end
    m = mean(x(:), 'omitnan');
end
