% analyze_muca_v0_hold_allpoints.m — release (V0) vs hold-phase reading,
% per iteration, for all 19 points
% =========================================================================
% CORRECTED METHODOLOGY: V0 is the mean of the raw 'locate'-phase samples
% (probe at the surface, pressing=False -- TOTAL RELEASE), not a minimum
% pulled out of the hold phase. The hold reading is the mean of the raw
% 'hold'-phase samples (the actual loaded measurement) -- both computed
% straight from the logged per-sample rows (muca_release_hold_stats.m).
%
%   V0(point, surface, round)   = mean(V) over that round's 'locate' rows
%                                  @ depth_mm = 5 (shallowest swept depth)
%   Hold(point, surface, round) = mean(V) over that round's 'hold' rows
%                                  @ depth_mm = 5
%   deltaV(round)               = Hold(round) - V0(round)
%
% No log file has both 19-point coverage AND 10 rounds -- the only files
% with all 19 points are the 5-round *_5iters_*/long_dataset_* logs
% (round_idx 0-4), same three files estimate_capacitance.m uses.
%
% Outputs
% -------
%   1. Console + CSV: full per-iteration table
%   2. Console + CSV: per (surface, point) summary (n, mean/median/std/min/max
%      of V0, Hold and delta_v)
%   3. Hex maps: median V0, median Hold, median delta-V per point, all 19
%      points, 3 panels (flat/solid/hollow).
%
% Usage
% -----
%   octave-cli analyze_muca_v0_hold_allpoints.m
%   (or open/run in MATLAB)

clear; close all; clc;

% ── CONFIG ───────────────────────────────────────────────────────────────
FLAT_CSV   = 'flat_new_5iters_session_20260813_195924.csv';
SOLID_CSV  = 'long_dataset_solid_dome_session_20260812_192705.csv';
HOLLOW_CSV = 'hollow_dome_5iters_session_20260814_132756.csv';

MAX_ROUND = 5;    % round_idx 0..4 -- these logs do NOT have 10 rounds
DEPTH_MM  = 5;    % mm -- shallowest depth in the sweep

SURFACE_NAMES  = {'flat', 'solid', 'hollow'};
SURFACE_FILES  = {FLAT_CSV, SOLID_CSV, HOLLOW_CSV};

% ── Paths ────────────────────────────────────────────────────────────────
HERE        = fileparts(mfilename('fullpath'));
LOG_DIR     = fullfile(HERE, '..', 'logs');
RESULTS_DIR = fullfile(HERE, 'results');
if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end

[point_ids, point_xy, own_cell_col] = muca_layout();
HEX_RADIUS = 8.0 / sqrt(3);

% ── Load + compute per-iteration rows, all 19 points ────────────────────
% per_iter_rows columns: [surface_idx, point, round_idx,
%   locate_mean, locate_min, locate_max, locate_n,
%   hold_mean, hold_min, hold_max, hold_n, delta_v]
per_iter_rows = zeros(0, 12);

fprintf('%s\n', repmat('=', 1, 78));
fprintf('  MUCA-BOARD RELEASE (V0) vs HOLD reading -- all 19 points\n');
fprintf('  V0 = mean of raw ''locate''-phase samples @ %gmm (total release)\n', DEPTH_MM);
fprintf('  Hold = mean of raw ''hold''-phase samples @ %gmm (loaded)\n', DEPTH_MM);
fprintf('  NOTE: these logs have %d rounds (0-%d), not 10\n', MAX_ROUND, MAX_ROUND - 1);
fprintf('%s\n', repmat('=', 1, 78));

for s = 1:3
    csv_path = fullfile(LOG_DIR, SURFACE_FILES{s});
    fprintf('\n  %s : %s\n', upper(SURFACE_NAMES{s}), csv_path);
    if ~exist(csv_path, 'file')
        fprintf('    [warn] file not found -- skipping this surface\n');
        continue;
    end
    d = read_muca_csv(csv_path);

    for p = 1:19
        rows_p = muca_release_hold_stats(d, p, own_cell_col, MAX_ROUND, DEPTH_MM);
        n = size(rows_p, 1);
        if n < MAX_ROUND
            fprintf('    [warn] point %d: found %d/%d rounds with both locate+hold data\n', p, n, MAX_ROUND);
        end
        for i = 1:n
            per_iter_rows(end + 1, :) = [s, p, rows_p(i, :)]; %#ok<AGROW>
        end
    end
end

% ── 1) Full per-iteration table -- console + CSV ────────────────────────
fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('  PER-ITERATION TABLE\n');
fprintf('%s\n', repmat('=', 1, 78));
fprintf('  %-8s %6s %6s | %9s %8s %8s %4s | %9s %8s %8s %4s | %8s\n', ...
    'surface', 'point', 'round', 'V0_mean', 'V0_min', 'V0_max', 'n', ...
    'Hold_mean', 'Hold_min', 'Hold_max', 'n', 'delta_V');
for i = 1:size(per_iter_rows, 1)
    s = per_iter_rows(i, 1);
    r = per_iter_rows(i, 3:end);
    fprintf('  %-8s %6d %6d | %9.4f %8.4f %8.4f %4d | %9.4f %8.4f %8.4f %4d | %8.4f\n', ...
        SURFACE_NAMES{s}, per_iter_rows(i, 2), r(1), r(2), r(3), r(4), r(5), r(6), r(7), r(8), r(9), r(10));
end

csv_path_iter = fullfile(RESULTS_DIR, 'muca_v0_hold_allpoints_per_iteration.csv');
fid = fopen(csv_path_iter, 'w');
fprintf(fid, 'surface,point,round_idx,locate_mean,locate_min,locate_max,locate_n,hold_mean,hold_min,hold_max,hold_n,delta_v\n');
for i = 1:size(per_iter_rows, 1)
    s = per_iter_rows(i, 1);
    fprintf(fid, '%s,%d,%d,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%d,%.6f\n', SURFACE_NAMES{s}, ...
        per_iter_rows(i, 2), per_iter_rows(i, 3), per_iter_rows(i, 4), per_iter_rows(i, 5), ...
        per_iter_rows(i, 6), per_iter_rows(i, 7), per_iter_rows(i, 8), per_iter_rows(i, 9), ...
        per_iter_rows(i, 10), per_iter_rows(i, 11), per_iter_rows(i, 12));
end
fclose(fid);
fprintf('\n  saved: %s\n', csv_path_iter);

% ── 2) Per (surface, point) summary across iterations ───────────────────
% summary_rows columns: [surface_idx, point, n,
%   v0_mean, v0_median, v0_std, v0_min, v0_max,
%   hold_mean, hold_median, hold_std, hold_min, hold_max,
%   dv_mean, dv_median, dv_std, dv_min, dv_max]
summary_rows = zeros(0, 18);
for s = 1:3
    for p = 1:19
        m = (per_iter_rows(:, 1) == s) & (per_iter_rows(:, 2) == p);
        if ~any(m)
            continue;
        end
        v0   = per_iter_rows(m, 4);
        hold = per_iter_rows(m, 8);
        dv   = per_iter_rows(m, 12);
        summary_rows(end + 1, :) = [s, p, numel(v0), ...
            mean(v0), median(v0), std(v0), min(v0), max(v0), ...
            mean(hold), median(hold), std(hold), min(hold), max(hold), ...
            mean(dv), median(dv), std(dv), min(dv), max(dv)]; %#ok<AGROW>
    end
end

fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('  SUMMARY ACROSS ITERATIONS  (per surface, per point)\n');
fprintf('%s\n', repmat('=', 1, 78));
fprintf('  %-8s %6s %4s | %8s %8s %8s | %8s %8s %8s | %8s %8s %8s\n', ...
    'surface', 'point', 'n', 'V0_mean', 'V0_med', 'V0_std', ...
    'Hold_mean', 'Hold_med', 'Hold_std', 'dV_mean', 'dV_med', 'dV_std');
for i = 1:size(summary_rows, 1)
    s = summary_rows(i, 1);
    fprintf('  %-8s %6d %4d | %8.4f %8.4f %8.4f | %8.4f %8.4f %8.4f | %8.4f %8.4f %8.4f\n', ...
        SURFACE_NAMES{s}, summary_rows(i, 2), summary_rows(i, 3), ...
        summary_rows(i, 4), summary_rows(i, 5), summary_rows(i, 6), ...
        summary_rows(i, 9), summary_rows(i, 10), summary_rows(i, 11), ...
        summary_rows(i, 14), summary_rows(i, 15), summary_rows(i, 16));
end

csv_path_summary = fullfile(RESULTS_DIR, 'muca_v0_hold_allpoints_summary.csv');
fid = fopen(csv_path_summary, 'w');
fprintf(fid, 'surface,point,n,v0_mean,v0_median,v0_std,v0_min,v0_max,hold_mean,hold_median,hold_std,hold_min,hold_max,dv_mean,dv_median,dv_std,dv_min,dv_max\n');
for i = 1:size(summary_rows, 1)
    s = summary_rows(i, 1);
    fprintf(fid, '%s,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', SURFACE_NAMES{s}, ...
        summary_rows(i, 2), summary_rows(i, 3), summary_rows(i, 4), summary_rows(i, 5), summary_rows(i, 6), ...
        summary_rows(i, 7), summary_rows(i, 8), summary_rows(i, 9), summary_rows(i, 10), summary_rows(i, 11), ...
        summary_rows(i, 12), summary_rows(i, 13), summary_rows(i, 14), summary_rows(i, 15), summary_rows(i, 16), ...
        summary_rows(i, 17), summary_rows(i, 18));
end
fclose(fid);
fprintf('\n  saved: %s\n', csv_path_summary);

% ── 3) Hex maps -- median V0 / Hold / delta-V per point, full 19-point coverage ──
hex_v0   = nan(19, 3);
hex_hold = nan(19, 3);
hex_dv   = nan(19, 3);
for i = 1:size(summary_rows, 1)
    s = summary_rows(i, 1);
    p = summary_rows(i, 2);
    hex_v0(p, s)   = summary_rows(i, 5);
    hex_hold(p, s) = summary_rows(i, 10);
    hex_dv(p, s)   = summary_rows(i, 15);
end

function save_hexmap(hex_vals, point_ids, point_xy, HEX_RADIUS, SURFACE_NAMES, cb_label, title_str, out_path)
    all_vals = hex_vals(~isnan(hex_vals));
    if isempty(all_vals)
        fprintf('  [warn] no values for "%s" -- skipping hex map\n', title_str);
        return;
    end
    vmin = min(all_vals);
    vmax = max(all_vals);
    cmap = get_cmap();
    fig = figure('Position', [100 100 1500 550]);
    panel_left  = [0.03, 0.35, 0.67];
    panel_width = 0.27;
    axes_list = cell(1, 3);
    for s = 1:3
        ax = axes('Parent', fig, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
        axes_list{s} = ax;
        draw_hex_panel(ax, point_ids, point_xy, hex_vals(:, s), cmap, vmin, vmax, ...
            HEX_RADIUS, upper(SURFACE_NAMES{s}));
    end
    colormap(fig, cmap);
    for s = 1:3
        set(axes_list{s}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, cb_label);
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
    end
    fig_suptitle(fig, title_str);
    save_fig(fig, out_path);
end

save_hexmap(hex_v0, point_ids, point_xy, HEX_RADIUS, SURFACE_NAMES, ...
    sprintf('V0 -- release, mean of locate-phase samples @ %gmm (median of %d iterations)', DEPTH_MM, MAX_ROUND), ...
    sprintf('Muca-Board -- V0 (total release) per point, all 19 points (median of %d iterations)', MAX_ROUND), ...
    fullfile(RESULTS_DIR, 'muca_v0_hold_allpoints_hexmap'));

save_hexmap(hex_hold, point_ids, point_xy, HEX_RADIUS, SURFACE_NAMES, ...
    sprintf('Hold -- mean of hold-phase samples @ %gmm (median of %d iterations)', DEPTH_MM, MAX_ROUND), ...
    sprintf('Muca-Board -- Hold reading per point, all 19 points (median of %d iterations)', MAX_ROUND), ...
    fullfile(RESULTS_DIR, 'muca_hold_allpoints_hexmap'));

save_hexmap(hex_dv, point_ids, point_xy, HEX_RADIUS, SURFACE_NAMES, ...
    sprintf('delta-V = Hold - V0 @ %gmm (median of %d iterations)', DEPTH_MM, MAX_ROUND), ...
    sprintf('Muca-Board -- delta-V (Hold - V0) per point, all 19 points (median of %d iterations)', MAX_ROUND), ...
    fullfile(RESULTS_DIR, 'muca_deltaV_allpoints_hexmap'));

fprintf('\nFigures + CSVs saved in: %s/\n', RESULTS_DIR);
