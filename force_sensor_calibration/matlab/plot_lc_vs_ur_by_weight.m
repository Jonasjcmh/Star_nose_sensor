%% plot_lc_vs_ur_by_weight.m
%
% MATLAB port of plot_lc_vs_ur_by_weight.py.
%
% Plots the load cell (LC) force and the UR robot's own force (fz) side
% by side, for each calibration weight, using only the sessions where the
% load cell was actually installed (fzcal_futek_direct_*). The
% fzcal_ur_only_* sessions have no load cell and are not used here.
%
% This script is self-contained (it does not call a1_fit_lc_ur_calibration.m)
% and repeats the same discovery / de-duplication / load-cell fit logic,
% so it can be read and run on its own -- exactly like its Python
% counterpart, which recomputes everything "self-contained, from the same
% raw files" rather than importing.
%
% The fit itself is calibrated on ABSOLUTE ai0/fz (baseline included as a
% known non-zero point -- the load cell + holder/hook hardware is already
% resting on the sensor during the no-load baseline too, so it isn't a
% zero reference). For each de-duplicated (direction, weight) session,
% using the LOADED phase's absolute readings:
%   F_lc        = lc_rate_N_per_V * ai0_load_mean + lc_offset_N   (load-cell force)
%   F_ur_raw    = fz_load_mean                                     (UR fz, raw)
%   F_ur_signed = ai0_sign(direction) * abs(fz_load_mean)          (UR fz, sign-
%                 corrected to the load cell's own push/pull convention)
%
% Two panels (posz / negz), grouped bars per weight, THREE bars each: LC,
% UR raw, and UR sign-corrected. The raw bar keeps the original sign
% mismatch visible (negz especially can come out the wrong way vs LC);
% the sign-corrected bar puts F_ur on the same sign convention as F_lc so
% the two magnitudes can actually be compared directly, bar to bar.
%
% Run this file directly (F5, or "run plot_lc_vs_ur_by_weight" from the
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

%% ---- Discover + de-duplicate the load-cell-installed sessions ----

entries = discover_entries(LOG_DIR, 'futek_direct', STANDARD_WEIGHTS_G);
entries = dedupe_latest(entries);

sessions_cell = cell(1, numel(entries));
for i = 1:numel(entries)
    sessions_cell{i} = load_session(entries(i), G);
end
sessions = sort_sessions([sessions_cell{:}]);

%% ---- Load-cell voltage <-> force fit (baseline + loaded as known points) ----

points = expand_phases(sessions);
ai0_pts = [points.ai0]';
fsigned_pts = [points.F_signed]';
[lc_rate_N_per_V, lc_offset_N, lc_r2, lc_rmse] = linfit(ai0_pts, fsigned_pts);

for i = 1:numel(sessions)
    sessions(i).F_lc = lc_rate_N_per_V * sessions(i).ai0_load_mean + lc_offset_N;
    sessions(i).F_ur_raw = sessions(i).fz_load_mean;
    sessions(i).F_ur_signed = ai0_sign(sessions(i).direction) * abs(sessions(i).fz_load_mean);
end

fprintf('LC voltage<->force fit used: F_lc = %.4f*ai0 + (%.5f)  (R^2=%.5f)\n', ...
        lc_rate_N_per_V, lc_offset_N, lc_r2);
fprintf('%8s%6s%10s%12s%13s%12s%13s\n', 'weight_g', 'dir', 'F_lc(N)', 'F_ur_raw(N)', ...
        'F_ur_sign(N)', 'diff_raw(N)', 'diff_sign(N)');
for i = 1:numel(sessions)
    s = sessions(i);
    fprintf('%8.0f%6s%10.4f%12.4f%13.4f%12.4f%13.4f\n', s.weight_g, s.direction, s.F_lc, ...
            s.F_ur_raw, s.F_ur_signed, s.F_ur_raw - s.F_lc, s.F_ur_signed - s.F_lc);
end

%% ---- Plot: grouped bars, one panel per direction ----

fig = figure('Color', 'w', 'Position', [100 100 1400 550]);
directions = {'posz', 'negz'};
bar_width = 0.26;
axes_handles = gobjects(1, 2);

for d = 1:2
    direction = directions{d};
    ax = subplot(1, 2, d);
    axes_handles(d) = ax;
    hold(ax, 'on');

    in_direction = strcmp({sessions.direction}, direction);
    d_sessions = sessions(in_direction);
    % already globally sorted by (direction, weight_g), so d_sessions is
    % sorted by weight_g here too.

    n = numel(d_sessions);
    x = 1:n;
    lc_vals = [d_sessions.F_lc];
    ur_raw_vals = [d_sessions.F_ur_raw];
    ur_signed_vals = [d_sessions.F_ur_signed];

    bar(ax, x - bar_width, lc_vals, bar_width, 'FaceColor', [0.10 0.43 0.71]);
    bar(ax, x, ur_raw_vals, bar_width, 'FaceColor', [0.84 0.15 0.16], 'FaceAlpha', 0.45);
    bar(ax, x + bar_width, ur_signed_vals, bar_width, 'FaceColor', [0.84 0.15 0.16]);

    yline(ax, 0, 'Color', 'k', 'LineWidth', 0.8);
    xticks(ax, x);
    labels = cell(1, n);
    for i = 1:n
        labels{i} = sprintf('%d g', round(d_sessions(i).weight_g));
    end
    xticklabels(ax, labels);
    xlabel(ax, 'Applied weight');
    title(ax, direction);
    grid(ax, 'off');
end

ylabel(axes_handles(1), 'Force (N), absolute (loaded phase)');
legend(axes_handles(1), {'LC (load cell)', 'UR robot (fz, raw)', 'UR robot (fz, sign-corrected)'}, ...
       'Location', 'northwest');
linkaxes(axes_handles, 'y');   % same y-scale in both panels, like the Python version

sgtitle(fig, sprintf(['LC vs UR robot force per weight -- futek\\_direct sessions (load cell installed)\n' ...
                      'UR sign-corrected: fz\\_signed = ai0\\_sign(direction)*|fz\\_raw|, same convention as the load cell']));

out_path = fullfile(OUT_DIR, 'lc_vs_ur_by_weight_matlab.png');
print(fig, out_path, '-dpng', '-r150');   % 'print' works on both old MATLAB and Octave
fprintf('\nSaved -> %s\n', out_path);


%% ============================= Local functions =============================
% Same helpers as a1_fit_lc_ur_calibration.m, repeated here so this script
% runs completely on its own (see the module docstring above for why).

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


function s = load_session(entry, G)
% LOAD_SESSION  Read one calibration CSV + its meta json, and compute the
% baseline and loaded means for fz and ai0 (this script only ever loads
% futek_direct sessions, which always have both). The baseline is a
% known, non-zero load (load cell + holder/hook hardware mass), not a
% zero reference -- see expand_phases.

    meta = jsondecode(fileread(entry.meta_path));
    T = readtable(entry.csv_path);

    is_loaded = T.loaded == 1;
    is_base = T.loaded == 0;

    s = entry;
    s.tilt_deg = meta.tilt_from_vertical_deg;
    s.fz_base_mean = mean(T.fz(is_base));
    s.fz_load_mean = mean(T.fz(is_loaded));
    s.ai0_base_mean = mean(T.ai0(is_base));
    s.ai0_load_mean = mean(T.ai0(is_loaded));

    tilt_rad = deg2rad(meta.tilt_from_vertical_deg);
    hardware_g = extra_hardware_futek_direct(entry.direction);
    total_g = entry.weight_g + hardware_g;

    s.F_true_base = (hardware_g / 1000) * G * cos(tilt_rad);
    s.F_true = (total_g / 1000) * G * cos(tilt_rad);
    s.F_signed = ai0_sign(entry.direction) * s.F_true;
    s.F_signed_base = ai0_sign(entry.direction) * s.F_true_base;
end


function points = expand_phases(sessions)
% EXPAND_PHASES  Turn futek_direct sessions into a flat struct array of
% per-phase points, 2 per session (baseline then loaded).

    points_cell = cell(1, 2 * numel(sessions));
    for i = 1:numel(sessions)
        s = sessions(i);

        b.weight_g = s.weight_g; b.direction = s.direction; b.phase = 'baseline';
        b.ai0 = s.ai0_base_mean; b.F_signed = s.F_signed_base;

        l.weight_g = s.weight_g; l.direction = s.direction; l.phase = 'loaded';
        l.ai0 = s.ai0_load_mean; l.F_signed = s.F_signed;

        points_cell{2 * i - 1} = b;
        points_cell{2 * i} = l;
    end
    points = [points_cell{:}];
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


function extra_g = extra_hardware_futek_direct(direction)
% EXTRA_HARDWARE_FUTEK_DIRECT  Hardware mass (g) felt by the load cell's
% OWN reading in the futek_direct rig, on top of the nominal test weight:
% only what's mounted ABOVE the load cell in the load path -- the holder
% (7 g, posz) or the hook (4 g, negz). Not the load cell body or the UR
% mount below it.

    if strcmp(direction, 'posz')
        extra_g = 7;
    else
        extra_g = 4;
    end
end
