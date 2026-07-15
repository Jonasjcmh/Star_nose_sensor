%% plot_ur_only_vs_load.m
%
% MATLAB port of plot_ur_only_vs_load.py.
%
% Plots the UR wrist sensor (fz) against the known load used for testing,
% using ONLY the fzcal_ur_only_* sessions (no load cell installed in this
% chain at all). This isolates the question "does the robot's own force
% sensor track a known weight", independent of any load-cell fit.
%
% This script is self-contained, like the other MATLAB scripts in this
% folder -- it repeats the same discovery / de-duplication / baseline-
% compensation logic rather than depending on another file.
%
% The baseline (loaded==0) is NOT a zero reference here: the attachment
% (15 g) + 4 screws (21 g) + holder (7 g, posz) or hook (1 g, negz) are
% already resting on the sensor during that phase too, so it's a known,
% non-zero load in its own right. Each de-duplicated (direction, weight)
% session therefore contributes TWO absolute (fz, F_true) points, not one
% baseline-compensated delta:
%   (fz_base_mean, F_true_base = hardware only)
%   (fz_load_mean, F_true      = hardware + weight_g)
%
% Fits F_true = a*fz_ur + b over all these points, pooled AND separately
% per direction -- a pooled fit across both directions is only meaningful
% if they actually agree. Also fits a SIGN-CORRECTED pooled version, using
% fz_signed = ai0_sign(direction) * abs(fz_raw) AND F_signed =
% ai0_sign(direction) * F_true (same convention as a1_fit_lc_ur_calibration.m)
% -- BOTH sides need the same push/pull sign convention, not just fz, or
% pooling makes the fit worse instead of better -- and plots it as a 3rd
% panel with marker SHAPE carrying the direction (circle=posz, square=negz).
%
% Run this file directly (F5, or "run plot_ur_only_vs_load" from the
% force_sensor_calibration/matlab folder). No toolboxes required.

clear; clc; close all;

%% ---- Paths & constants ----

HERE = fileparts(mfilename('fullpath'));
CALIB_DIR = fileparts(HERE);
LOG_DIR = fullfile(CALIB_DIR, 'logs');
OUT_DIR = fullfile(CALIB_DIR, 'plots');
if ~exist(OUT_DIR, 'dir')
    mkdir(OUT_DIR);
end

G = 9.80665;
STANDARD_WEIGHTS_G = [5 10 20 50 100 200];

%% ---- Discover + de-duplicate the ur_only sessions ----

entries = discover_entries(LOG_DIR, 'ur_only', STANDARD_WEIGHTS_G);
entries = dedupe_latest(entries);

sessions_cell = cell(1, numel(entries));
for i = 1:numel(entries)
    sessions_cell{i} = load_session(entries(i), G);
end
sessions = sort_sessions([sessions_cell{:}]);

fprintf('%8s%6s%9s%11s%11s\n', 'weight_g', 'dir', 'phase', 'fz_abs(N)', 'F_true(N)');
for i = 1:numel(sessions)
    s = sessions(i);
    fprintf('%8.0f%6s%9s%11.4f%11.4f\n', s.weight_g, s.direction, 'baseline', ...
            s.fz_base_mean, s.F_true_base);
    fprintf('%8.0f%6s%9s%11.4f%11.4f\n', s.weight_g, s.direction, 'loaded', ...
            s.fz_load_mean, s.F_true);
end

%% ---- Pooled fit (both directions together, both phases) ----

fz_all = reshape([[sessions.fz_base_mean]; [sessions.fz_load_mean]], [], 1);
f_true_all = reshape([[sessions.F_true_base]; [sessions.F_true]], [], 1);
[pooled_slope, pooled_offset, pooled_r2, pooled_rmse] = linfit(fz_all, f_true_all);
fprintf('\nn = %d points (%d sessions x 2 phases each)\n', numel(fz_all), numel(sessions));
fprintf('pooled:  F_true = %.4f * fz_robot + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        pooled_slope, pooled_offset, pooled_r2, pooled_rmse);

%% ---- Per-direction fit ----

is_posz = strcmp({sessions.direction}, 'posz');
is_negz = strcmp({sessions.direction}, 'negz');
% expand the per-session logical mask to the 2-points-per-session layout
is_posz2 = reshape([is_posz; is_posz], [], 1);
is_negz2 = reshape([is_negz; is_negz], [], 1);

[posz_slope, posz_offset, posz_r2, posz_rmse] = linfit(fz_all(is_posz2), f_true_all(is_posz2));
[negz_slope, negz_offset, negz_r2, negz_rmse] = linfit(fz_all(is_negz2), f_true_all(is_negz2));

fprintf('   posz: F_true = %.4f * fz_robot + (%.5f)   R^2 = %.5f   RMSE = %.4f N   (n=%d points)\n', ...
        posz_slope, posz_offset, posz_r2, posz_rmse, sum(is_posz2));
fprintf('   negz: F_true = %.4f * fz_robot + (%.5f)   R^2 = %.5f   RMSE = %.4f N   (n=%d points)\n', ...
        negz_slope, negz_offset, negz_r2, negz_rmse, sum(is_negz2));

%% ---- Sign-corrected pooled fit: fz AND ground truth both oriented by ----
%% direction (F_signed = ai0_sign(direction)*F_true), so the two sides
% actually share a sign convention -- pooling fz_signed against the
% unsigned F_true was the bug that made an earlier version of this fit
% worse than the raw one.

f_signed_all = reshape([[sessions.F_signed_base]; [sessions.F_signed]], [], 1);
fz_signed_all = zeros(size(fz_all));
fz_signed_all(is_posz2) = ai0_sign('posz') * abs(fz_all(is_posz2));
fz_signed_all(is_negz2) = ai0_sign('negz') * abs(fz_all(is_negz2));
[signed_slope, signed_offset, signed_r2, signed_rmse] = linfit(fz_signed_all, f_signed_all);
fprintf('\nsign-corrected pooled (fz_signed = ai0_sign(direction)*abs(fz_raw), F_signed = ai0_sign(direction)*F_true):\n');
fprintf('  F_signed = %.4f * fz_signed + (%.5f)   R^2 = %.5f   RMSE = %.4f N  (n=%d points, vs pooled-raw R^2 = %.5f)\n', ...
        signed_slope, signed_offset, signed_r2, signed_rmse, numel(fz_signed_all), pooled_r2);

%% ---- Plot: fz_abs vs F_true scatter, one panel per direction, plus a ----
%% 3rd panel with both directions pooled on the sign-corrected fz

fig = figure('Color', 'w', 'Position', [100 100 1900 550]);
directions = {'posz', 'negz'};
dir_fits = struct('posz', struct('slope', posz_slope, 'offset', posz_offset, 'r2', posz_r2), ...
                   'negz', struct('slope', negz_slope, 'offset', negz_offset, 'r2', negz_r2));
axes_handles = gobjects(1, 3);

for d = 1:2
    direction = directions{d};
    ax = subplot(1, 3, d);
    axes_handles(d) = ax;
    hold(ax, 'on');

    d_sessions = sessions(strcmp({sessions.direction}, direction));

    for i = 1:numel(d_sessions)
        s = d_sessions(i);
        color = weight_color(s.weight_g);
        plot(ax, s.fz_load_mean, s.F_true, 'o', 'Color', color, ...
             'MarkerFaceColor', color, 'MarkerSize', 9);
        plot(ax, s.fz_base_mean, s.F_true_base, 's', 'Color', color, ...
             'MarkerFaceColor', 'none', 'LineWidth', 1.5, 'MarkerSize', 7);
    end

    fit_d = dir_fits.(direction);
    fz_dir = [d_sessions.fz_base_mean, d_sessions.fz_load_mean];
    x_range = linspace(min(fz_dir) * 1.1, max(fz_dir) * 1.1, 200);
    plot(ax, x_range, fit_d.slope * x_range + fit_d.offset, 'k-', 'LineWidth', 2);

    yline(ax, 0, 'Color', [0.5 0.5 0.5]);
    xline(ax, 0, 'Color', [0.5 0.5 0.5]);
    title(ax, sprintf('%s   (fit: F_{true}=%.3f*fz+%.3f, R^2=%.4f)', ...
          direction, fit_d.slope, fit_d.offset, fit_d.r2));
    xlabel(ax, 'fz_{ur}, absolute (N)   [circle=loaded, open square=baseline]');
    grid(ax, 'on');
end

% --- Panel 3: both directions pooled, on sign-corrected fz -- marker
% SHAPE now carries the directionality (circle=posz, square=negz), since
% fz_signed already puts both directions on the same sign axis ---
ax3 = subplot(1, 3, 3);
axes_handles(3) = ax3;
hold(ax3, 'on');

for i = 1:numel(sessions)
    s = sessions(i);
    color = weight_color(s.weight_g);
    if strcmp(s.direction, 'posz')
        marker = 'o';
    else
        marker = 's';
    end
    fz_signed_load = ai0_sign(s.direction) * abs(s.fz_load_mean);
    fz_signed_base = ai0_sign(s.direction) * abs(s.fz_base_mean);
    plot(ax3, fz_signed_load, s.F_signed, marker, 'Color', color, ...
         'MarkerFaceColor', color, 'MarkerSize', 9);
    plot(ax3, fz_signed_base, s.F_signed_base, marker, 'Color', color, ...
         'MarkerFaceColor', 'none', 'LineWidth', 1.5, 'MarkerSize', 7);
end

x_range = linspace(min(fz_signed_all) * 1.1, max(fz_signed_all) * 1.1, 200);
plot(ax3, x_range, signed_slope * x_range + signed_offset, 'k-', 'LineWidth', 2);
yline(ax3, 0, 'Color', [0.5 0.5 0.5]);
xline(ax3, 0, 'Color', [0.5 0.5 0.5]);
title(ax3, sprintf('both directions, sign-corrected fz vs sign-corrected ground truth\n(fit: F_{signed}=%.3f*fz\\_signed+%.3f, R^2=%.4f)', ...
      signed_slope, signed_offset, signed_r2));
xlabel(ax3, ['fz_{signed} = ai0\_sign(direction)*|fz_{ur}| (N)   [circle=posz, square=negz]' ...
             newline '[filled=loaded, open=baseline]']);
ylabel(ax3, 'F_{signed} (N) = ai0\_sign(direction)*F_{true}');
grid(ax3, 'on');

ylabel(axes_handles(1), 'F_{true} (N)   [hardware only at baseline, hardware+weight when loaded]');
linkaxes(axes_handles(1:2), 'y');

sgtitle(fig, 'UR sensor (fz, absolute) vs known load -- ur\_only sessions (no load cell installed)');

out_path = fullfile(OUT_DIR, 'ur_only_vs_load_matlab.png');
print(fig, out_path, '-dpng', '-r150');
fprintf('\nSaved -> %s\n', out_path);


%% ============================= Local functions =============================
% Same helpers as the other MATLAB scripts in this folder, repeated here
% so this script runs completely on its own.

function entries = discover_entries(log_dir, instrument, standard_weights_g)
% DISCOVER_ENTRIES  Find fzcal_<instrument>_<direction>_<weight>g_<ts>.csv
% files and parse direction/weight/timestamp out of each filename.

    files = dir(fullfile(log_dir, sprintf('fzcal_%s_*.csv', instrument)));
    expr = ['fzcal_' instrument '_(?<direction>posz|negz)_' ...
            '(?<weight>\d+(\.\d+)?)g_(?<ts>\d{8}_\d{6})\.csv$'];

    entries_cell = {};
    for k = 1:numel(files)
        tok = regexp(files(k).name, expr, 'names');
        if isempty(tok)
            continue
        end
        if is_excluded_session(instrument, tok.direction, tok.ts)
            continue
        end
        weight_g = str2double(tok.weight);
        [~, nearest_idx] = min(abs(standard_weights_g - weight_g));

        e.instrument = instrument;
        e.direction = tok.direction;
        e.weight_g = weight_g;
        e.nominal_weight_g = standard_weights_g(nearest_idx);
        e.ts = tok.ts;
        e.csv_path = fullfile(files(k).folder, files(k).name);
        e.meta_path = strrep(e.csv_path, '.csv', '_meta.json');

        entries_cell{end + 1} = e; %#ok<AGROW>
    end

    if isempty(entries_cell)
        entries = struct([]);
    else
        entries = [entries_cell{:}];
    end
end


function tf = is_excluded_session(instrument, direction, ts)
% IS_EXCLUDED_SESSION  Manual exclusion, confirmed by whoever ran the
% collection: these two ur_only negz files (labeled 201g and 202g) are
% one-off mislabeled attempts that don't represent a real, distinct test
% point -- ignore them entirely. The plain 20g file
% (fzcal_ur_only_negz_20g_..._180618.csv) is used for the 20g point
% instead.

    tf = strcmp(instrument, 'ur_only') && strcmp(direction, 'negz') && ...
         (strcmp(ts, '20260706_181552') || strcmp(ts, '20260706_181726'));
end


function kept = dedupe_latest(entries)
% DEDUPE_LATEST  Group entries by (instrument, direction, nominal weight)
% and keep only the chronologically-last file in each group.

    keys = cell(1, numel(entries));
    for i = 1:numel(entries)
        keys{i} = sprintf('%s|%s|%g', entries(i).instrument, entries(i).direction, ...
                           entries(i).nominal_weight_g);
    end
    unique_keys = unique(keys);

    kept_cell = cell(1, numel(unique_keys));
    for k = 1:numel(unique_keys)
        idx = find(strcmp(keys, unique_keys{k}));
        group = entries(idx);
        [~, order] = sort({group.ts});
        group = group(order);

        if numel(group) > 1
            dropped_labels = cell(1, numel(group) - 1);
            for j = 1:numel(group) - 1
                dropped_labels{j} = sprintf('%gg@%s', group(j).weight_g, group(j).ts);
            end
            fprintf('[dedupe] %-12s %-4s %5.0fg: keeping %gg@%s (dropping %s)\n', ...
                    group(end).instrument, group(end).direction, group(end).nominal_weight_g, ...
                    group(end).weight_g, group(end).ts, strjoin(dropped_labels, ', '));
        end
        kept_cell{k} = group(end);
    end

    if isempty(kept_cell)
        kept = struct([]);
    else
        kept = [kept_cell{:}];
    end
end


function s = load_session(entry, G)
% LOAD_SESSION  Read one calibration CSV + its meta json, and compute
% the baseline-compensated delta for fz (ur_only sessions have no load
% cell, so ai0 is not read here).

    meta = jsondecode(fileread(entry.meta_path));
    T = readtable(entry.csv_path);

    is_loaded = T.loaded == 1;
    is_base = T.loaded == 0;

    fz_base = mean(T.fz(is_base));
    dfz = T.fz(is_loaded) - fz_base;

    s = entry;
    s.tilt_deg = meta.tilt_from_vertical_deg;
    s.dfz_mean = mean(dfz);
    s.dfz_std = std(dfz);
    s.fz_base_mean = fz_base;
    s.fz_load_mean = mean(T.fz(is_loaded));

    hardware_g = extra_hardware_ur_only(entry.direction);
    total_g = entry.weight_g + hardware_g;
    tilt_rad = deg2rad(meta.tilt_from_vertical_deg);
    s.F_true_base = (hardware_g / 1000) * G * cos(tilt_rad);
    s.F_true = (total_g / 1000) * G * cos(tilt_rad);
    % F_signed/F_signed_base orient the ground truth by measurement
    % direction (push=posz vs pull=negz), same ai0_sign convention used to
    % sign fz. Needed so a sign-corrected fz can be pooled against a
    % ground truth that's ALSO on the same sign convention.
    s.F_signed = ai0_sign(entry.direction) * s.F_true;
    s.F_signed_base = ai0_sign(entry.direction) * s.F_true_base;
end


function sorted = sort_sessions(sessions)
% SORT_SESSIONS  Order a session struct array by direction then weight_g.

    keys = cell(1, numel(sessions));
    for i = 1:numel(sessions)
        keys{i} = sprintf('%s_%08.2f', sessions(i).direction, sessions(i).weight_g);
    end
    [~, order] = sort(keys);
    sorted = sessions(order);
end


function [m, c, r2, rmse] = linfit(x, y)
% LINFIT  Ordinary least-squares line  y = m*x + c , plus R^2 and RMSE.

    x = x(:);
    y = y(:);
    p = polyfit(x, y, 1);
    m = p(1);
    c = p(2);

    y_pred = m * x + c;
    ss_res = sum((y - y_pred) .^ 2);
    ss_tot = sum((y - mean(y)) .^ 2);
    r2 = 1 - ss_res / ss_tot;
    rmse = sqrt(mean((y - y_pred) .^ 2));
end


function sgn = ai0_sign(direction)
% AI0_SIGN  Load-cell voltage sign convention: +z (posz) pushes the
% bridge voltage down, -z (negz) pulls it up (see a1_fit_lc_ur_calibration.m).
% Reused here to give fz_signed the same push/pull convention as F_true.

    if strcmp(direction, 'posz')
        sgn = -1;
    else
        sgn = 1;
    end
end


function extra_g = extra_hardware_ur_only(direction)
% EXTRA_HARDWARE_UR_ONLY  Hardware mass (g) felt by the UR sensor in this
% rig, on top of the nominal test weight: 3D-printed attachment (15 g) +
% 4 screws (21 g) are common to both directions, plus the holder (7 g,
% posz) or the hook (1 g, negz).

    if strcmp(direction, 'posz')
        extra_g = 15 + 21 + 7;
    else
        extra_g = 15 + 21 + 1;
    end
end


function c = weight_color(weight_g)
% WEIGHT_COLOR  Fixed color per nominal calibration weight, so the same
% weight always plots in the same color across figures.

    switch round(weight_g)
        case 5,   c = [0.12 0.47 0.71];
        case 10,  c = [1.00 0.50 0.05];
        case 20,  c = [0.17 0.63 0.17];
        case 50,  c = [0.84 0.15 0.16];
        case 100, c = [0.58 0.40 0.74];
        case 200, c = [0.55 0.34 0.29];
        otherwise, c = [0.2 0.2 0.2];
    end
end
