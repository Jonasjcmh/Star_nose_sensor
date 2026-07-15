%% plot_ur_only_compensation_crosscheck.m
%
% MATLAB port of plot_ur_only_compensation_crosscheck.py.
%
% Cross-validates the two UR compensation formulas fit in
% plot_lc_ur_force_vs_time.m (from the futek_direct rig: load cell + UR
% together, sign-corrected fz, pooled -200..+200 g) against the
% INDEPENDENT fzcal_ur_only_* dataset (UR sensor alone, different
% session, different hardware stack). This is genuine held-out
% validation: the coefficients below were fit on futek_direct data and
% are here applied, unmodified, to data they have never seen.
%
% Coefficients under test (hardcoded from plot_lc_ur_force_vs_time.m's
% printed "COEFFICIENTS SUMMARY", items #3 and #5 -- re-run that script if
% the underlying logs ever change):
%   #3  F_lc   = 0.7284 * fz_signed + (-0.20451)   [UR vs load cell]
%   #5  F_true = 1.0010 * fz_signed + (-0.27483)   [UR vs known weight]
% where fz_signed = ai0_sign(direction) * |fz_raw|, same sign-correction
% convention used throughout (negative for posz, positive for negz).
%
% This script is self-contained, like the other MATLAB scripts in this
% folder -- it repeats the same discovery / de-duplication / fit logic.
%
% For every raw sample (baseline + loaded) of every ur_only session,
% plots F_true (ground truth: known weight + ur_only's own hardware mass)
% against fz_signed, with the two transferred correction lines (#3, #5)
% evaluated as-is (no refitting), plus a fresh fit on the ur_only data
% itself as a best-case reference. Reports each transferred correction's
% real prediction RMSE on this held-out dataset.
%
% Run this file directly (F5, or "run plot_ur_only_compensation_crosscheck"
% from the force_sensor_calibration/matlab folder). No toolboxes required.

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

% Coefficients under test, from plot_lc_ur_force_vs_time.m (futek_direct,
% sign-corrected fz, pooled -200..+200 g). See script header.
COEFF_VS_FLC = [0.7284, -0.20451];      % item #3: F_lc   = a*fz_signed + b
COEFF_VS_FTRUE = [1.0010, -0.27483];    % item #5: F_true = a*fz_signed + b

%% ---- Discover + de-duplicate the ur_only sessions ----

entries = discover_entries(LOG_DIR, 'ur_only', STANDARD_WEIGHTS_G);
entries = dedupe_latest(entries);

sessions_cell = cell(1, numel(entries));
for i = 1:numel(entries)
    sessions_cell{i} = load_session(entries(i), G);
end
sessions = sort_sessions([sessions_cell{:}]);

%% ---- Build the held-out dataset: every raw sample, sign-corrected fz ----

fz_signed_all = [];
f_true_all = [];
meta_dir = {};
meta_weight = [];

for i = 1:numel(sessions)
    s = sessions(i);
    entry = find_entry(entries, s.direction, s.weight_g);
    [is_loaded, fz] = load_raw_series(entry.csv_path);
    sign_d = ai0_sign(s.direction);
    fz_signed = sign_d * abs(fz);
    f_true_base_signed = sign_d * s.F_true_base;

    n_base = sum(~is_loaded);
    n_load = sum(is_loaded);
    fz_signed_all = [fz_signed_all; fz_signed(~is_loaded); fz_signed(is_loaded)]; %#ok<AGROW>
    f_true_all = [f_true_all; repmat(f_true_base_signed, n_base, 1); repmat(s.F_signed, n_load, 1)]; %#ok<AGROW>
    meta_dir = [meta_dir, repmat({s.direction}, 1, n_base + n_load)]; %#ok<AGROW>
    meta_weight = [meta_weight; repmat(s.weight_g, n_base + n_load, 1)]; %#ok<AGROW>
end

n = numel(fz_signed_all);
fprintf('ur_only dataset: %d raw samples (%d sessions x ~%d samples each)\n', ...
        n, numel(sessions), round(n / numel(sessions)));

%% ---- Apply the two TRANSFERRED corrections, unmodified, and score them
%     against this held-out data (real prediction error, not a fit R^2) ----

pred_flc = COEFF_VS_FLC(1) * fz_signed_all + COEFF_VS_FLC(2);
pred_ftrue = COEFF_VS_FTRUE(1) * fz_signed_all + COEFF_VS_FTRUE(2);
rmse_flc = sqrt(mean((pred_flc - f_true_all) .^ 2));
rmse_ftrue = sqrt(mean((pred_ftrue - f_true_all) .^ 2));
bias_flc = mean(pred_flc - f_true_all);
bias_ftrue = mean(pred_ftrue - f_true_all);

fprintf('\nTransferred correction #3 (fit on F_lc, futek_direct) applied to ur_only:\n');
fprintf('  F_pred = %.4f*fz_signed + (%.5f)\n', COEFF_VS_FLC(1), COEFF_VS_FLC(2));
fprintf('  prediction RMSE = %.4f N   bias = %+.4f N   (n=%d)\n', rmse_flc, bias_flc, n);

fprintf('\nTransferred correction #5 (fit on F_true, futek_direct) applied to ur_only:\n');
fprintf('  F_pred = %.4f*fz_signed + (%.5f)\n', COEFF_VS_FTRUE(1), COEFF_VS_FTRUE(2));
fprintf('  prediction RMSE = %.4f N   bias = %+.4f N   (n=%d)\n', rmse_ftrue, bias_ftrue, n);

%% ---- Fresh fit on ur_only itself, as a best-case reference ----

[own_a, own_b, own_r2, own_rmse] = linfit(fz_signed_all, f_true_all);
fprintf('\nur_only''s own fit (best case for this rig/session):\n');
fprintf('  F_true = %.4f*fz_signed + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', own_a, own_b, own_r2, own_rmse);

%% ---- Plot ----

fig = figure('Color', 'w', 'Position', [100 100 1100 900]);
ax = axes(fig);
hold(ax, 'on');

directions = {'posz', 'negz'};
for d = 1:2
    direction = directions{d};
    for weight_g = STANDARD_WEIGHTS_G
        mask = strcmp(meta_dir, direction)' & (meta_weight == weight_g);
        if ~any(mask)
            continue
        end
        color = weight_color(weight_g);
        if strcmp(direction, 'posz')
            marker = 'o';
        else
            marker = 's';
        end
        scatter(ax, fz_signed_all(mask), f_true_all(mask), 10, color, 'filled', ...
                'MarkerFaceAlpha', 0.25, 'Marker', marker);
    end
end

margin = 0.05 * (max(fz_signed_all) - min(fz_signed_all));
x_range = linspace(min(fz_signed_all) - margin, max(fz_signed_all) + margin, 200);
h_own = plot(ax, x_range, own_a * x_range + own_b, 'k-', 'LineWidth', 2.5, ...
    'DisplayName', sprintf('ur\\_only''s own fit: F=%.3f*fz+%.3f (R^2=%.4f)', own_a, own_b, own_r2));
h_flc = plot(ax, x_range, COEFF_VS_FLC(1) * x_range + COEFF_VS_FLC(2), '--', 'Color', [0.10 0.43 0.71], 'LineWidth', 2, ...
    'DisplayName', sprintf('transferred #3 (vs F_lc): F=%.3f*fz+%.3f (RMSE on ur\\_only=%.3f N)', ...
    COEFF_VS_FLC(1), COEFF_VS_FLC(2), rmse_flc));
h_ftrue = plot(ax, x_range, COEFF_VS_FTRUE(1) * x_range + COEFF_VS_FTRUE(2), ':', 'Color', [0.84 0.15 0.16], 'LineWidth', 2, ...
    'DisplayName', sprintf('transferred #5 (vs F\\_true): F=%.3f*fz+%.3f (RMSE on ur\\_only=%.3f N)', ...
    COEFF_VS_FTRUE(1), COEFF_VS_FTRUE(2), rmse_ftrue));

yline(ax, 0, 'Color', [0.5 0.5 0.5]);
xline(ax, 0, 'Color', [0.5 0.5 0.5]);
xlabel(ax, 'fz_{signed} = ai0\_sign(direction)*|fz_{raw}| (N) -- UR sensor, ur\_only sessions');
ylabel(ax, 'F_{true} (N) -- known weight + hardware, signed');
title(ax, sprintf(['UR compensation cross-check -- transferred coefficients (from futek\\_direct)' newline ...
      'evaluated on the independent ur\\_only dataset (held-out, no load cell)']));
grid(ax, 'off');

weight_values = [5 10 20 50 100 200];
weight_handles = gobjects(1, numel(weight_values));
for k = 1:numel(weight_values)
    weight_handles(k) = plot(ax, NaN, NaN, 'o', 'Color', weight_color(weight_values(k)), ...
        'MarkerFaceColor', weight_color(weight_values(k)), 'MarkerSize', 8, ...
        'DisplayName', sprintf('%d g', weight_values(k)));
end
legend(ax, [weight_handles, h_own, h_flc, h_ftrue], 'FontSize', 8, 'NumColumns', 1, 'Location', 'northwest');

out_path = fullfile(OUT_DIR, 'ur_only_compensation_crosscheck_matlab.png');
print(fig, out_path, '-dpng', '-r150');
fprintf('\nSaved -> %s\n', out_path);


%% ============================= Local functions =============================

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
% point -- ignore them entirely. The plain 20g file is used instead.

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


function entry = find_entry(entries, direction, weight_g)
% FIND_ENTRY  Return the single deduped entry matching (direction,
% nominal weight), or [] if none exists.

    for i = 1:numel(entries)
        if strcmp(entries(i).direction, direction) && entries(i).nominal_weight_g == weight_g
            entry = entries(i);
            return
        end
    end
    entry = [];
end


function s = load_session(entry, G)
% LOAD_SESSION  Read one calibration CSV + its meta json, and compute the
% baseline and loaded means for fz. The baseline is a known, non-zero
% load (attachment/screws/holder-hook hardware mass), not a zero
% reference.

    meta = jsondecode(fileread(entry.meta_path));
    T = readtable(entry.csv_path);

    is_loaded = T.loaded == 1;
    is_base = T.loaded == 0;

    s = entry;
    s.tilt_deg = meta.tilt_from_vertical_deg;
    s.fz_base_mean = mean(T.fz(is_base));
    s.fz_load_mean = mean(T.fz(is_loaded));

    tilt_rad = deg2rad(meta.tilt_from_vertical_deg);
    hardware_g = extra_hardware_ur_only(entry.direction);
    total_g = entry.weight_g + hardware_g;

    s.F_true_base = (hardware_g / 1000) * G * cos(tilt_rad);
    s.F_true = (total_g / 1000) * G * cos(tilt_rad);
    s.F_signed = ai0_sign(entry.direction) * s.F_true;
end


function [is_loaded, fz] = load_raw_series(csv_path)
% LOAD_RAW_SERIES  Read the raw loaded flag and fz column for one session CSV.

    T = readtable(csv_path);
    is_loaded = T.loaded == 1;
    fz = T.fz;
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
% bridge voltage down, -z (negz) pulls it up.

    if strcmp(direction, 'posz')
        sgn = -1;
    else
        sgn = 1;
    end
end


function extra_g = extra_hardware_ur_only(direction)
% EXTRA_HARDWARE_UR_ONLY  Hardware mass (g) felt by the UR sensor in the
% ur_only (no load cell) rig, on top of the nominal test weight: 43 g
% (posz) / 37 g (negz).

    if strcmp(direction, 'posz')
        extra_g = 43;
    else
        extra_g = 37;
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
