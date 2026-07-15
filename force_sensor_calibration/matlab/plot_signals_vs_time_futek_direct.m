%% plot_signals_vs_time_futek_direct.m
%
% Raw UR fz AND load-cell ai0 vs time, both signals in the same panel, one
% panel per (direction, weight), for the fzcal_futek_direct_* sessions
% (load cell installed alongside the UR robot -- fz and ai0 are logged
% together, same rows, same timestamps). 2 rows (posz, negz) x 6 columns
% (5/10/20/50/100/200 g), same weight in the same column on both rows so
% the two directions line up for comparison.
%
% This is the futek_direct counterpart of plot_fz_vs_time_ur_only.m, which
% covers the no-load-cell sessions. This script is self-contained, like
% the other MATLAB scripts in this folder -- it repeats the same
% discovery / de-duplication logic.
%
% Each panel shows ONLY the loaded window (loaded==1), time re-zeroed to
% the start of that window. Two y-axes (yyaxis): left = fz (N, UR robot),
% right = ai0 (V, load cell). Every panel shares the SAME left-axis range
% and the SAME right-axis range (each sized to the largest range seen
% across all sessions for that signal), so magnitudes are directly
% comparable panel to panel. No gridlines. The title reports both
% baseline-compensated deltas (dFz and dV) for a quick read.
%
% The left (fz) axis also shows the EXPECTED force level, i.e. F_true_ur =
% (weight_g + extra_hardware_futek_direct_ur(direction)) worth of force --
% NOT just the nominal weight_g, and NOT the load cell's own hardware
% total either: the UR holds up everything below its flange in this rig,
% including the load cell's own body (50 g posz / 47 g negz total), which
% is more than what the load cell itself feels (7 g / 4 g). Drawn as two
% symmetric dashed lines at +F_true_ur and -F_true_ur, same reasoning as
% plot_fz_vs_time_ur_only.m: raw fz's sign isn't assumed, both possible
% signs are shown.
%
% Run this file directly (F5, or "run plot_signals_vs_time_futek_direct"
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

STANDARD_WEIGHTS_G = [5 10 20 50 100 200];
G = 9.80665;

%% ---- Discover + de-duplicate the futek_direct sessions ----

entries = discover_entries(LOG_DIR, 'futek_direct', STANDARD_WEIGHTS_G);
entries = dedupe_latest(entries);

%% ---- Pass 1: load every panel's loaded-window data, track global ranges ----

directions = {'posz', 'negz'};
panel_data = cell(2, numel(STANDARD_WEIGHTS_G));
fz_lo = Inf;  fz_hi = -Inf;
ai0_lo = Inf; ai0_hi = -Inf;

for row = 1:2
    direction = directions{row};
    for col = 1:numel(STANDARD_WEIGHTS_G)
        weight_g = STANDARD_WEIGHTS_G(col);
        entry = find_entry(entries, direction, weight_g);
        if isempty(entry)
            continue
        end

        [t, is_loaded, fz, ai0] = load_raw_series(entry.csv_path);
        fz_base_mean = mean(fz(~is_loaded));
        fz_load_mean = mean(fz(is_loaded));
        ai0_base_mean = mean(ai0(~is_loaded));
        ai0_load_mean = mean(ai0(is_loaded));

        load_start = t(find(is_loaded, 1, 'first'));
        t_load = t(is_loaded) - load_start;
        fz_load = fz(is_loaded);
        ai0_load = ai0(is_loaded);

        meta = jsondecode(fileread(entry.meta_path));
        tilt_rad = deg2rad(meta.tilt_from_vertical_deg);
        hardware_g_ur = extra_hardware_futek_direct_ur(direction);
        total_g_ur = weight_g + hardware_g_ur;
        f_true_ur = (total_g_ur / 1000) * G * cos(tilt_rad);

        panel_data{row, col} = struct('t', t_load, 'fz', fz_load, 'ai0', ai0_load, ...
            'weight_g', weight_g, 'fz_base_mean', fz_base_mean, 'fz_load_mean', fz_load_mean, ...
            'ai0_base_mean', ai0_base_mean, 'ai0_load_mean', ai0_load_mean, ...
            'f_true_ur', f_true_ur, 'total_g_ur', total_g_ur);

        fz_lo = min([fz_lo, min(fz_load), -f_true_ur]);   fz_hi = max([fz_hi, max(fz_load), f_true_ur]);
        ai0_lo = min(ai0_lo, min(ai0_load)); ai0_hi = max(ai0_hi, max(ai0_load));
    end
end

fz_margin = 0.05 * (fz_hi - fz_lo);
ai0_margin = 0.05 * (ai0_hi - ai0_lo);
fz_range = [fz_lo - fz_margin, fz_hi + fz_margin];
ai0_range = [ai0_lo - ai0_margin, ai0_hi + ai0_margin];

fprintf('%6s%10s%12s%9s%15s\n', 'dir', 'weight_g', 'hardware_g', 'real_g', 'F_true_ur(N)');
for row = 1:2
    direction = directions{row};
    for col = 1:numel(STANDARD_WEIGHTS_G)
        d = panel_data{row, col};
        if isempty(d)
            continue
        end
        hardware_g_ur = extra_hardware_futek_direct_ur(direction);
        fprintf('%6s%10.0f%12.0f%9.0f%15.4f\n', direction, d.weight_g, hardware_g_ur, ...
                d.total_g_ur, d.f_true_ur);
    end
end

%% ---- Pass 2: plot every panel with the shared ranges ----

fig = figure('Color', 'w', 'Position', [50 50 3000 900]);

for row = 1:2
    direction = directions{row};
    for col = 1:numel(STANDARD_WEIGHTS_G)
        idx = (row - 1) * numel(STANDARD_WEIGHTS_G) + col;
        ax = subplot(2, numel(STANDARD_WEIGHTS_G), idx);

        d = panel_data{row, col};
        if isempty(d)
            set(ax, 'Visible', 'off');
            continue
        end

        color = weight_color(d.weight_g);

        yyaxis(ax, 'left');
        hold(ax, 'on');
        plot(ax, d.t, d.fz, 'Color', color, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        h_mean = yline(ax, d.fz_load_mean, 'Color', [0.20 0.20 0.20], 'LineStyle', '--');
        h_mean.DisplayName = sprintf('measured mean=%.3f N', d.fz_load_mean);
        h_exp = yline(ax, d.f_true_ur, 'Color', [0.17 0.63 0.17], 'LineStyle', ':', 'LineWidth', 1.3);
        h_exp.DisplayName = sprintf('expected +-%.3f N (%d g real)', d.f_true_ur, round(d.total_g_ur));
        h_exp2 = yline(ax, -d.f_true_ur, 'Color', [0.17 0.63 0.17], 'LineStyle', ':', 'LineWidth', 1.3);
        h_exp2.HandleVisibility = 'off';
        ylim(ax, fz_range);
        ylabel(ax, 'fz (N)', 'FontSize', 8);
        legend(ax, [h_mean, h_exp], 'Location', 'southeast', 'FontSize', 5);

        yyaxis(ax, 'right');
        hold(ax, 'on');
        plot(ax, d.t, d.ai0, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
        yline(ax, d.ai0_load_mean, 'Color', [0.60 0.20 0.05], 'LineStyle', ':');
        ylim(ax, ai0_range);
        ylabel(ax, 'ai0 (V)', 'FontSize', 8);

        title(ax, sprintf('%s -- %d g nominal (%d g real w/ hardware)\ndFz=%+.3f N, dV=%+.4f V   F_true_ur=+-%.3f N', ...
              direction, d.weight_g, round(d.total_g_ur), d.fz_load_mean - d.fz_base_mean, ...
              d.ai0_load_mean - d.ai0_base_mean, d.f_true_ur), 'FontSize', 8);
        if row == 2
            xlabel(ax, 'time since load start (s)', 'FontSize', 8);
        end
        grid(ax, 'off');
    end
end

sgtitle(fig, ['UR fz (left axis) + load-cell ai0 (right axis) vs time, loaded window only' newline ...
              '-- futek\_direct sessions   |   rows: posz (top) / negz (bottom)   |   same y-scale across all panels']);

out_path = fullfile(OUT_DIR, 'futek_direct_signals_vs_time_matlab.png');
print(fig, out_path, '-dpng', '-r150');
fprintf('Saved -> %s\n', out_path);


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


function [t, is_loaded, fz, ai0] = load_raw_series(csv_path)
% LOAD_RAW_SERIES  Read the full raw time series for one session CSV --
% both fz (UR robot) and ai0 (load cell), which futek_direct logs together.

    T = readtable(csv_path);
    t = T.timestamp - T.timestamp(1);
    is_loaded = T.loaded == 1;
    fz = T.fz;
    ai0 = T.ai0;
end


function extra_g = extra_hardware_futek_direct_ur(direction)
% EXTRA_HARDWARE_FUTEK_DIRECT_UR  Hardware mass (g) felt by the UR sensor
% itself in the futek_direct rig: the UR holds up everything below it in
% the load path -- the 3D-printed coupler (15 g) + 4 attachment screws
% (21 g) + the load cell's own body (7 g) = 43 g, common to both
% directions, plus the holder (7 g, posz) or the hook (4 g, negz) above
% the load cell. This is MORE than what the load cell itself feels
% (7 g posz / 4 g negz, see a1_fit_lc_ur_calibration.m's
% extra_hardware_futek_direct), since the UR also carries the load
% cell's own body weight.

    if strcmp(direction, 'posz')
        extra_g = 50;
    else
        extra_g = 47;
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
