% analyze_interdome.m — Interdome_touch dataset analysis & visualization (MATLAB/Octave)
% =========================================================================
% MATLAB/Octave port of analyze_interdome.py. Reads a CSV produced by
% main.py (and its companion *_meta.json, if present) and reconstructs:
%
%   1. The UR5 TCP trajectory — XY path over the sensor grid, and Z (depth)
%      vs time so the press/hold/retract waveform is visible.
%   2. Sample counts (press/hold/retract) broken down by depth, point_kind
%      (type), and point (label).
%   3. The 19-cell hexagonal schematic, colored by capacitive sensor
%      intensity (own cell, hold-phase mean) and FUTEK force (hold-phase
%      mean), one hex grid per depth plus an all-depths summary.
%
% Defaults to ../logs/interdome_20260724_162417.csv — change CSV_NAME below
% to analyze a different run. Figures are saved into results/ (this folder).
%
% Lives in Interdome_touch/matlab_analysis/ -- one level below
% Interdome_touch/ itself (which holds logs/, plots/, and the Python
% analyze_interdome.py this was ported from).
%
% Tested with GNU Octave 10 (conda env `octave_test`); should also run
% unmodified in MATLAB (readtable/io-package dependency deliberately
% avoided — CSV is parsed with textscan, which is core to both).
%
% Usage
% -----
%   octave-cli analyze_interdome.m
%   (or open/run in MATLAB)

clear; close all; clc;

CSV_NAME = 'interdome_20260724_162417';   % <- change to analyze a different run

% ── Paths ─────────────────────────────────────────────────────────────────
HERE             = fileparts(mfilename('fullpath'));
INTERDOME_DIR    = fullfile(HERE, '..');
INTEGRATION_DIR  = fullfile(INTERDOME_DIR, '..', 'Integration_2');
LOG_DIR          = fullfile(INTERDOME_DIR, 'logs');
PLOTS_DIR        = fullfile(HERE, 'results');
DEFAULT_POINTS_JSON = fullfile(INTEGRATION_DIR, 'calib_points_supposed_rigid_transformed.json');

HEX_RADIUS   = 8.0 / sqrt(3);   % ~4.6188 mm, matches Integration_2/plot_rigid.py
ANCHOR_POINT = 10;

PHASE_NAMES  = {'locate', 'press', 'hold', 'retract', 'post'};
PHASE_RGB    = [0.122 0.467 0.706;   % locate  #1f77b4
                 0.839 0.153 0.157;  % press   #d62728
                 0.173 0.627 0.173;  % hold    #2ca02c
                 1.000 0.498 0.055;  % retract #ff7f0e
                 0.580 0.404 0.741]; % post    #9467bd

SAMPLE_PHASES  = {'press', 'hold', 'retract'};
PHASE_LABELS   = {'down ramp (press)', 'hold', 'up ramp (retract)'};

if ~exist(PLOTS_DIR, 'dir')
    mkdir(PLOTS_DIR);
end

csv_path  = fullfile(LOG_DIR, [CSV_NAME '.csv']);
meta_path = fullfile(LOG_DIR, [CSV_NAME '_meta.json']);
if ~exist(csv_path, 'file')
    error('CSV not found: %s', csv_path);
end

fprintf('[load] %s\n', csv_path);

% ── Meta ─────────────────────────────────────────────────────────────────
meta = struct();
if exist(meta_path, 'file')
    meta = jsondecode(fileread(meta_path));
else
    fprintf('[warn] No companion meta file at %s -- using defaults\n', meta_path);
end

% ── Points geometry (rotated upright about the anchor point, display-only) ─
points_json = DEFAULT_POINTS_JSON;
if isfield(meta, 'points_source') && exist(meta.points_source, 'file')
    points_json = meta.points_source;
end
if ~exist(points_json, 'file')
    points_json = DEFAULT_POINTS_JSON;
end
pj = jsondecode(fileread(points_json));

rotation_deg = 0.0;
if isfield(pj, 'rotation_deg')
    rotation_deg = pj.rotation_deg;
end
anchor = ANCHOR_POINT;
if isfield(pj, 'anchor_point')
    anchor = pj.anchor_point;
end

pt_fields = fieldnames(pj.points);
n_pts = numel(pt_fields);
point_ids = zeros(n_pts, 1);
point_xy  = zeros(n_pts, 2);
for i = 1:n_pts
    fname = pt_fields{i};              % e.g. 'x10' for JSON key "10"
    point_ids(i) = str2double(fname(2:end));
    point_xy(i, 1) = pj.points.(fname).x_mm;
    point_xy(i, 2) = pj.points.(fname).y_mm;
end
[point_ids, order] = sort(point_ids);
point_xy = point_xy(order, :);

if rotation_deg ~= 0
    pivot = point_xy(point_ids == anchor, :);
    theta = deg2rad(-rotation_deg);
    c = cos(theta); s = sin(theta);
    rot = [c, -s; s, c];
    point_xy = (point_xy - pivot) * rot' + pivot;
end

% ── Load CSV (textscan — no io/readtable package dependency) ───────────────
fid = fopen(csv_path, 'r');
header_line = fgetl(fid);
header = strsplit(header_line, ',');
n_cols = numel(header);
if n_cols ~= 41
    fprintf('[warn] expected 41 CSV columns, found %d -- column layout may differ\n', n_cols);
end

fmt = ['%f' '%*s' repmat('%f', 1, 3) repmat('%s', 1, 3) repmat('%f', 1, 14) repmat('%f', 1, 19)];
C = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
fclose(fid);

timestamp       = C{1};
depth_mm        = C{3};
point           = C{5};
point_kind      = C{6};
phase           = C{7};
point_x_mm      = C{8};
point_y_mm      = C{9};
tcp_z           = C{13};
futek_force_N   = C{21};
cells           = [C{22:40}];   % N x 19, cell_1..cell_19

n_rows = numel(timestamp);

depths_mm = [];
if isfield(meta, 'depths_mm')
    depths_mm = meta.depths_mm(:);
end
if isempty(depths_mm)
    depths_mm = unique(depth_mm);
end
depths_mm = sort(depths_mm);

iterations = '?';
if isfield(meta, 'iterations')
    iterations = meta.iterations;
end

fprintf('  rows       : %d\n', n_rows);
fprintf('  depths     : %s mm\n', mat2str(depths_mm'));
if ischar(iterations)
    fprintf('  iterations : %s\n', iterations);
else
    fprintf('  iterations : %d\n', iterations);
end
fprintf('  points     : %s\n', mat2str(point_ids'));

base = CSV_NAME;

% ── 1) Trajectory ────────────────────────────────────────────────────────
% Subsample rows for the scatter plots only (hex/sample-count/summary stats
% below still use the full dataset). With ~1e6 rows, plotting every point as
% an individual vector marker makes the SVG export hundreds of MB for no
% visual benefit (massive overplotting either way) -- so cap the plotted
% point count for both PNG and SVG.
TRAJ_MAX_POINTS = 30000;
traj_stride = max(1, floor(n_rows / TRAJ_MAX_POINTS));
traj_idx = (1:traj_stride:n_rows)';
traj_phase = phase(traj_idx);
traj_x = point_x_mm(traj_idx);
traj_y = point_y_mm(traj_idx);
traj_z = tcp_z(traj_idx);
traj_t = timestamp(traj_idx) - timestamp(1);

fig1 = figure('Position', [100 100 1500 650]);

ax1 = subplot(1, 2, 1);
hold(ax1, 'on');
for i = 1:numel(point_ids)
    [vx, vy] = hex_vertices(point_xy(i, 1), point_xy(i, 2), HEX_RADIUS);
    patch(ax1, vx, vy, 'w', 'FaceColor', 'none', 'EdgeColor', [0.73 0.73 0.73], 'LineWidth', 1);
    text(ax1, point_xy(i, 1), point_xy(i, 2), num2str(point_ids(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontSize', 7, 'Color', [0.53 0.53 0.53]);
end
legend_handles = [];
legend_labels  = {};
for p = 1:numel(PHASE_NAMES)
    m = strcmp(traj_phase, PHASE_NAMES{p});
    if any(m)
        h = scatter_safe(ax1, traj_x(m), traj_y(m), 8, PHASE_RGB(p, :), 0.35);
        legend_handles(end + 1) = h; %#ok<SAGROW>
        legend_labels{end + 1}  = PHASE_NAMES{p}; %#ok<SAGROW>
    end
end
axis(ax1, 'equal'); grid(ax1, 'on');
xlabel(ax1, 'x (mm)'); ylabel(ax1, 'y (mm)');
title(ax1, 'Reconstructed trajectory - points visited (by phase)', 'FontWeight', 'bold');
if ~isempty(legend_handles)
    legend(ax1, legend_handles, legend_labels, 'Location', 'northeast', 'FontSize', 8);
end

ax2 = subplot(1, 2, 2);
hold(ax2, 'on');
for p = 1:numel(PHASE_NAMES)
    m = strcmp(traj_phase, PHASE_NAMES{p});
    if any(m)
        scatter_safe(ax2, traj_t(m), traj_z(m) * 1000.0, 4, PHASE_RGB(p, :), 0.5);
    end
end
grid(ax2, 'on');
xlabel(ax2, 'time (s)'); ylabel(ax2, 'tcp_z (mm)', 'Interpreter', 'none');
title(ax2, 'TCP height vs time (mm)', 'FontWeight', 'bold');
if ~isempty(legend_labels)
    legend(ax2, legend_labels, 'Location', 'northeast', 'FontSize', 8);
end

fig_suptitle(fig1, 'Interdome_touch - trajectory reconstruction');
save_fig(fig1, fullfile(PLOTS_DIR, [base '_trajectory_matlab']));

% ── 2) Sample counts — per depth / type (point_kind) / label (point) ───────
fprintf('\n');
fprintf('%s\n', repmat('=', 1, 70));
fprintf('  SAMPLE COUNTS (press=down ramp, hold, retract=up ramp)\n');
fprintf('%s\n', repmat('=', 1, 70));

count_groups = {depth_mm, point_kind, point};
count_titles = {'depth (depth_mm)', 'type (point_kind)', 'label (point)'};
count_results = cell(1, 3);

for gi = 1:3
    fprintf('\n  -- per %s --\n', count_titles{gi});
    [groups, counts, totals] = count_by_phase(count_groups{gi}, phase, SAMPLE_PHASES);
    count_results{gi} = {groups, counts, totals};
    fprintf('  %-14s%18s%18s%18s%10s\n', '', PHASE_LABELS{1}, PHASE_LABELS{2}, PHASE_LABELS{3}, 'total');
    for i = 1:numel(totals)
        if iscell(groups)
            g_str = groups{i};
        else
            g_str = sprintf('%.1f', groups(i));
        end
        fprintf('  %-14s%18d%18d%18d%10d\n', g_str, counts(i, 1), counts(i, 2), counts(i, 3), totals(i));
    end
end
fprintf('%s\n', repmat('=', 1, 70));

fig2 = figure('Position', [100 100 2800 750]);
panel_titles  = {'Samples per depth (mm)', 'Samples per type (point_kind)', 'Samples per label (point)'};
panel_rot     = [0, 0, 90];
panel_tickfs  = [8, 8, 5.5];
% [left width] per panel, normalized figure units -- label panel (85
% categories) gets far more room than depth/type (5 and 4 categories).
panel_left    = [0.03, 0.24, 0.45];
panel_width   = [0.17, 0.17, 0.53];

for gi = 1:3
    ax = axes('Parent', fig2, 'Position', [panel_left(gi), 0.18, panel_width(gi), 0.68]);
    set(ax, 'ColorOrder', PHASE_RGB([2 3 4], :), 'NextPlot', 'replacechildren');
    groups = count_results{gi}{1};
    counts = count_results{gi}{2};
    n = numel(groups);
    bar(ax, 1:n, counts, 'grouped');
    set(ax, 'YScale', 'log');
    set(ax, 'XTick', 1:n, 'FontSize', panel_tickfs(gi));
    if iscell(groups)
        set(ax, 'XTickLabel', groups);
    else
        set(ax, 'XTickLabel', arrayfun(@(v) sprintf('%.1f', v), groups, 'UniformOutput', false));
    end
    set(ax, 'TickLabelInterpreter', 'none');   % labels like 'D10_5' aren't TeX subscripts
    xtickangle(ax, panel_rot(gi));
    xlim(ax, [0, n + 1]);
    ylabel(ax, 'sample count (log scale)', 'FontSize', 8);
    title(ax, panel_titles{gi}, 'FontWeight', 'bold', 'Interpreter', 'none', 'FontSize', 9);
    legend(ax, PHASE_LABELS, 'Location', 'northeast', 'FontSize', 7);
    grid(ax, 'on');
end
fig_suptitle(fig2, 'Interdome_touch - sample counts by phase');
save_fig(fig2, fullfile(PLOTS_DIR, [base '_sample_counts_matlab']));

% ── 3/4) Hex schematics — capacitive sensor + FUTEK force (hold phase) ─────
main_mask = strcmp(point_kind, 'main');
hold_mask = strcmp(phase, 'hold');

point_num_id = nan(n_rows, 1);
idx_to_parse = find(main_mask & hold_mask);
for k = 1:numel(idx_to_parse)
    r = idx_to_parse(k);
    point_num_id(r) = parse_main_point_id(point{r});
end

cap_own_col = nan(19, 1);
for pt = 1:19
    cap_own_col(pt) = own_cell_index(pt);
end

n_depths = numel(depths_mm);
cap_stats    = nan(19, n_depths);
futek_stats  = nan(19, n_depths);
cap_var      = nan(19, n_depths);
futek_var    = nan(19, n_depths);
cap_median   = nan(19, n_depths);
futek_median = nan(19, n_depths);
for di = 1:n_depths
    d_mask = hold_mask & main_mask & (depth_mm == depths_mm(di));
    for pt = 1:19
        pt_mask = d_mask & (point_num_id == pt);
        if any(pt_mask)
            col = cap_own_col(pt);
            if ~isnan(col)
                cap_stats(pt, di)  = mean(cells(pt_mask, col));
                cap_var(pt, di)    = var(cells(pt_mask, col));
                cap_median(pt, di) = median(cells(pt_mask, col));
            end
            futek_stats(pt, di)  = mean(futek_force_N(pt_mask));
            futek_var(pt, di)    = var(futek_force_N(pt_mask));
            futek_median(pt, di) = median(futek_force_N(pt_mask));
        end
    end
end

plot_hex_by_depth(cap_stats, point_ids, point_xy, depths_mm, HEX_RADIUS, ...
    fullfile(PLOTS_DIR, [base '_hex_capacitive_matlab']), ...
    'Interdome_touch - capacitive sensor intensity (own cell, hold-phase mean)', ...
    'normalized capacitive value (0-1)', get_cmap('capacitive'));

plot_hex_by_depth(futek_stats, point_ids, point_xy, depths_mm, HEX_RADIUS, ...
    fullfile(PLOTS_DIR, [base '_hex_futek_matlab']), ...
    'Interdome_touch - FUTEK load cell force (hold-phase mean)', ...
    'force (N)', get_cmap('force'));

% ── 5) Variance per point — stability of the hold-phase (press) reading ────
% One grouped bar chart per point (P01-P19), bars = depth, height = variance
% of the hold-phase capacitive/FUTEK reading at that point/depth. Low
% variance = a stable/repeatable press; high variance flags a noisy point.
depth_legend = arrayfun(@(v) sprintf('%.1f mm', v), depths_mm, 'UniformOutput', false);

fig3 = figure('Position', [100 100 2200 850]);

point_xtick_labels = arrayfun(@(v) sprintf('P%02d', v), point_ids, 'UniformOutput', false);

ax = axes('Parent', fig3, 'Position', [0.05, 0.60, 0.85, 0.30]);
bar(ax, 1:19, cap_var, 'grouped');
set(ax, 'XTick', 1:19, 'XTickLabel', point_xtick_labels, 'TickLabelInterpreter', 'none');
xlim(ax, [0, 20]);
ylabel(ax, 'variance (capacitive)');
title(ax, 'Hold-phase capacitive variance per point', 'FontWeight', 'bold');
legend(ax, depth_legend, 'Location', 'northeastoutside', 'FontSize', 7);
grid(ax, 'on');

ax = axes('Parent', fig3, 'Position', [0.05, 0.08, 0.85, 0.30]);
bar(ax, 1:19, futek_var, 'grouped');
set(ax, 'XTick', 1:19, 'XTickLabel', point_xtick_labels, 'TickLabelInterpreter', 'none');
xlim(ax, [0, 20]);
ylabel(ax, 'variance (FUTEK force, N^2)');
title(ax, 'Hold-phase FUTEK force variance per point', 'FontWeight', 'bold');
legend(ax, depth_legend, 'Location', 'northeastoutside', 'FontSize', 7);
grid(ax, 'on');

fig_suptitle(fig3, 'Interdome_touch - per-point variance of the stable (hold-phase) reading');
save_fig(fig3, fullfile(PLOTS_DIR, [base '_point_variance_matlab']));

% ── 6) Median per point — typical stable (hold-phase) reading ──────────────
% Same layout as the variance chart above, but height = median (robust to
% outliers, unlike the mean already shown in the hex schematics) of the
% hold-phase capacitive/FUTEK reading at that point/depth.
fig4 = figure('Position', [100 100 2200 850]);

ax = axes('Parent', fig4, 'Position', [0.05, 0.60, 0.85, 0.30]);
bar(ax, 1:19, cap_median, 'grouped');
set(ax, 'XTick', 1:19, 'XTickLabel', point_xtick_labels, 'TickLabelInterpreter', 'none');
xlim(ax, [0, 20]);
ylabel(ax, 'median (capacitive)');
title(ax, 'Hold-phase capacitive median per point', 'FontWeight', 'bold');
legend(ax, depth_legend, 'Location', 'northeastoutside', 'FontSize', 7);
grid(ax, 'on');

ax = axes('Parent', fig4, 'Position', [0.05, 0.08, 0.85, 0.30]);
bar(ax, 1:19, futek_median, 'grouped');
set(ax, 'XTick', 1:19, 'XTickLabel', point_xtick_labels, 'TickLabelInterpreter', 'none');
xlim(ax, [0, 20]);
ylabel(ax, 'median (FUTEK force, N)');
title(ax, 'Hold-phase FUTEK force median per point', 'FontWeight', 'bold');
legend(ax, depth_legend, 'Location', 'northeastoutside', 'FontSize', 7);
grid(ax, 'on');

fig_suptitle(fig4, 'Interdome_touch - per-point median of the stable (hold-phase) reading');
save_fig(fig4, fullfile(PLOTS_DIR, [base '_point_median_matlab']));

% ── 7) Variance + median per point for the interpolated point kinds ────────
% diagonal/triangle/horizontal points sit between main hex cells, so they
% don't have a single "own" capacitive cell the way P01-P19 do (that mapping
% -- own_cell_index.m -- only covers the 19 main points) -- FUTEK force has
% no such ambiguity, so these charts are FUTEK-only.
extra_kinds = {'diagonal', 'triangle', 'horizontal'};
for ki = 1:numel(extra_kinds)
    kind_name = extra_kinds{ki};
    kind_mask = strcmp(point_kind, kind_name);
    if ~any(kind_mask)
        continue;
    end
    [kind_labels, kind_var, kind_median] = label_stats_by_depth( ...
        point, kind_mask, hold_mask, depth_mm, depths_mm, futek_force_N);
    n_labels = numel(kind_labels);

    fig_w = max(1200, 300 + n_labels * 55);
    figk = figure('Position', [100 100 fig_w 850]);

    ax = axes('Parent', figk, 'Position', [0.05, 0.60, 0.90, 0.30]);
    bar(ax, 1:n_labels, kind_var, 'grouped');
    set(ax, 'XTick', 1:n_labels, 'XTickLabel', kind_labels, 'FontSize', 6.5, 'TickLabelInterpreter', 'none');
    xtickangle(ax, 90);
    xlim(ax, [0, n_labels + 1]);
    ylabel(ax, 'variance (FUTEK force, N^2)');
    title(ax, sprintf('Hold-phase FUTEK force variance per %s point', kind_name), 'FontWeight', 'bold');
    legend(ax, depth_legend, 'Location', 'northeastoutside', 'FontSize', 7);
    grid(ax, 'on');

    ax = axes('Parent', figk, 'Position', [0.05, 0.08, 0.90, 0.30]);
    bar(ax, 1:n_labels, kind_median, 'grouped');
    set(ax, 'XTick', 1:n_labels, 'XTickLabel', kind_labels, 'FontSize', 6.5, 'TickLabelInterpreter', 'none');
    xtickangle(ax, 90);
    xlim(ax, [0, n_labels + 1]);
    ylabel(ax, 'median (FUTEK force, N)');
    title(ax, sprintf('Hold-phase FUTEK force median per %s point', kind_name), 'FontWeight', 'bold');
    legend(ax, depth_legend, 'Location', 'northeastoutside', 'FontSize', 7);
    grid(ax, 'on');

    fig_suptitle(figk, sprintf('Interdome_touch - %s points: FUTEK variance and median (hold phase)', kind_name));
    save_fig(figk, fullfile(PLOTS_DIR, [base '_' kind_name '_futek_var_median_matlab']));
end

% ── Summary table ────────────────────────────────────────────────────────
fprintf('\n');
fprintf('%s\n', repmat('=', 1, 70));
fprintf('  SUMMARY (hold-phase means)\n');
fprintf('%s\n', repmat('=', 1, 70));
fprintf('  %9s  %9s  %8s  %13s  %12s\n', 'depth_mm', 'mean_cap', 'max_cap', 'mean_futek_N', 'max_futek_N');
for di = 1:n_depths
    cap_vals = cap_stats(:, di);
    futek_vals = futek_stats(:, di);
    cap_vals = cap_vals(~isnan(cap_vals));
    futek_vals = futek_vals(~isnan(futek_vals));
    if isempty(cap_vals); mc = NaN; xc = NaN; else; mc = mean(cap_vals); xc = max(cap_vals); end
    if isempty(futek_vals); mf = NaN; xf = NaN; else; mf = mean(futek_vals); xf = max(futek_vals); end
    fprintf('  %9.1f  %9.3f  %8.3f  %13.3f  %12.3f\n', depths_mm(di), mc, xc, mf, xf);
end
fprintf('%s\n', repmat('=', 1, 70));
fprintf('\nFigures saved in: %s/\n', PLOTS_DIR);
