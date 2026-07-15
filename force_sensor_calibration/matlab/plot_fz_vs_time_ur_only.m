%% plot_fz_vs_time_ur_only.m
%
% MATLAB port of plot_fz_vs_time_ur_only.py.
%
% Raw UR fz vs time, one panel per (direction, weight), for the
% fzcal_ur_only_* sessions (no load cell installed). 2 rows (posz, negz)
% x 6 columns (5/10/20/50/100/200 g), same weight in the same column on
% both rows so the two directions line up for comparison.
%
% This script is self-contained, like the other MATLAB scripts in this
% folder -- it repeats the same discovery / de-duplication logic (so the
% same "use just the last one" rule applies).
%
% Each panel shows ONLY the loaded window (loaded==1), time re-zeroed to
% the start of that window, plus the loaded-phase mean as a dashed line.
% Every panel shares the SAME y-axis range -- sized to the largest range
% seen across all sessions -- so magnitudes are directly comparable
% panel to panel. No gridlines.
%
% Each panel also shows the EXPECTED force level, i.e. F_true = (weight_g
% + extra_hardware_ur_only(direction)) worth of force, NOT just the
% nominal weight_g -- the attachment/screws/holder-or-hook are already on
% the sensor too, so e.g. a nominal "5 g" posz point is really a ~48 g
% equivalent load. Drawn as two symmetric dashed lines at +F_true and
% -F_true: the raw fz sign convention isn't assumed here (posz and negz
% raw fz were both found to trend negative with load in this rig, unlike
% the load cell's own ai0_sign convention), so both possible signs are
% shown rather than guessing one.
%
% Run this file directly (F5, or "run plot_fz_vs_time_ur_only" from the
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

STANDARD_WEIGHTS_G = [5 10 20 50 100 200];
G = 9.80665;

%% ---- Discover + de-duplicate the ur_only sessions ----

entries = discover_entries(LOG_DIR, 'ur_only', STANDARD_WEIGHTS_G);
entries = dedupe_latest(entries);

%% ---- Pass 1: load every panel's loaded-window data, track global y-range ----

directions = {'posz', 'negz'};
panel_data = cell(2, numel(STANDARD_WEIGHTS_G));
global_lo = Inf;
global_hi = -Inf;

for row = 1:2
    direction = directions{row};
    for col = 1:numel(STANDARD_WEIGHTS_G)
        weight_g = STANDARD_WEIGHTS_G(col);
        entry = find_entry(entries, direction, weight_g);
        if isempty(entry)
            continue
        end

        [t, is_loaded, fz] = load_raw_series(entry.csv_path);
        fz_base_mean = mean(fz(~is_loaded));
        fz_load_mean = mean(fz(is_loaded));

        load_start = t(find(is_loaded, 1, 'first'));
        t_load = t(is_loaded) - load_start;   % re-zero to the start of the loaded window
        fz_load = fz(is_loaded);

        meta = jsondecode(fileread(entry.meta_path));
        tilt_rad = deg2rad(meta.tilt_from_vertical_deg);
        hardware_g = extra_hardware_ur_only(direction);
        total_g = weight_g + hardware_g;
        f_true = (total_g / 1000) * G * cos(tilt_rad);

        panel_data{row, col} = struct('t', t_load, 'fz', fz_load, 'weight_g', weight_g, ...
            'fz_base_mean', fz_base_mean, 'fz_load_mean', fz_load_mean, ...
            'f_true', f_true, 'total_g', total_g);

        global_lo = min([global_lo, min(fz_load), -f_true]);
        global_hi = max([global_hi, max(fz_load), f_true]);
    end
end

margin = 0.05 * (global_hi - global_lo);
y_range = [global_lo - margin, global_hi + margin];

fprintf('%6s%10s%12s%9s%12s\n', 'dir', 'weight_g', 'hardware_g', 'real_g', 'F_true(N)');
for row = 1:2
    direction = directions{row};
    for col = 1:numel(STANDARD_WEIGHTS_G)
        d = panel_data{row, col};
        if isempty(d)
            continue
        end
        hardware_g = extra_hardware_ur_only(direction);
        fprintf('%6s%10.0f%12.0f%9.0f%12.4f\n', direction, d.weight_g, hardware_g, d.total_g, d.f_true);
    end
end

%% ---- Pass 2: plot every panel with the shared y-range ----

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

        hold(ax, 'on');
        color = weight_color(d.weight_g);
        plot(ax, d.t, d.fz, 'Color', color, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        h_mean = yline(ax, d.fz_load_mean, 'Color', [0.20 0.20 0.20], 'LineStyle', '--');
        h_mean.DisplayName = sprintf('measured mean=%.3f N', d.fz_load_mean);
        h_exp = yline(ax, d.f_true, 'Color', [0.17 0.63 0.17], 'LineStyle', ':', 'LineWidth', 1.3);
        h_exp.DisplayName = sprintf('expected +-%.3f N (%d g real)', d.f_true, round(d.total_g));
        h_exp2 = yline(ax, -d.f_true, 'Color', [0.17 0.63 0.17], 'LineStyle', ':', 'LineWidth', 1.3);
        h_exp2.HandleVisibility = 'off';

        ylim(ax, y_range);
        title(ax, sprintf('%s -- %d g nominal (%d g real w/ hardware)\ndFz=%+.3f N   F_true=+-%.3f N', ...
              direction, d.weight_g, round(d.total_g), d.fz_load_mean - d.fz_base_mean, d.f_true), ...
              'FontSize', 8.5);
        legend(ax, [h_mean, h_exp], 'Location', 'southeast', 'FontSize', 5.5);
        if row == 2
            xlabel(ax, 'time since load start (s)', 'FontSize', 8);
        end
        if col == 1
            ylabel(ax, 'fz (N)', 'FontSize', 8);
        end
        grid(ax, 'off');
    end
end

sgtitle(fig, ['UR wrist fz vs time, loaded window only -- ur\_only sessions (no load cell)' newline ...
              'rows: posz (top) / negz (bottom)   |   same y-scale across all panels']);

out_path = fullfile(OUT_DIR, 'ur_only_fz_vs_time_matlab.png');
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


function [t, is_loaded, fz] = load_raw_series(csv_path)
% LOAD_RAW_SERIES  Read the full raw time series (not just the
% baseline/loaded means) for one session CSV.

    T = readtable(csv_path);
    t = T.timestamp - T.timestamp(1);
    is_loaded = T.loaded == 1;
    fz = T.fz;
end


function extra_g = extra_hardware_ur_only(direction)
% EXTRA_HARDWARE_UR_ONLY  Hardware mass (g) felt by the UR sensor in the
% ur_only (no load cell) rig, on top of the nominal test weight: 3D-
% printed attachment (15 g) + 4 screws (21 g) are common to both
% directions, plus the holder (7 g, posz) or the hook (1 g, negz).

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
