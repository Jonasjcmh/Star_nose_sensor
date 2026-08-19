% analyze_mucaboard.m — Muca-Board surface comparison (MATLAB/Octave)
% =========================================================================
% Simple 3-surface (flat / solid / hollow) comparison from
% mucaboard_data/logs/*.csv (produced by mucaboard_ramp_collector.py):
%
%   1. Force vs time during the press-hold-retract ramp, for a chosen depth
%      and number of iterations — one overlaid curve per surface.
%   2. Three 19-cell hex schematics (one per surface), each point coloured
%      by its own capacitive cell's hold-phase mean, on a shared colour
%      scale, for the same chosen depth and number of iterations.
%
% Edit the CONFIG block below: which CSV is which surface, which depth, and
% how many iterations (rounds) to include PER SURFACE (NUM_ITERATIONS is a
% [flat, solid, hollow] vector, since they don't all have the same number
% of rounds collected). Every plot reports how many iterations it actually
% found per surface (console + legend/panel titles as "(n=X)"), which can
% be less than requested if a surface/round is missing that depth/point.
%
% NOTE on current data (as of this writing): the flat/solid logs only
% contain depth_mm in {0, 10} (no intermediate ramp steps yet), and only
% round_idx 0 (one real iteration). The hollow log below additionally has
% depths 5-10 and 2 rounds. Console output at the top of a run always
% prints each surface's actually-available depths/rounds so you can pick a
% DEPTH_MM/NUM_ITERATIONS that exists in all three.
%
% Usage
% -----
%   octave-cli analyze_mucaboard.m
%   (or open/run in MATLAB)

clear; close all; clc;

% ── CONFIG ───────────────────────────────────────────────────────────────
FLAT_CSV   = 'flat_3points_session_20260813_180036.csv';
SOLID_CSV  = 'solid_3points_session_20260813_173548.csv';
HOLLOW_CSV = 'hollow_3points_session_20260813_181218.csv';

DEPTH_MM        = 10;         % which press depth to analyze/compare
NUM_ITERATIONS  = [10, 10, 10];   % per surface [flat, solid, hollow]: use
                                % round_idx 0 .. NUM_ITERATIONS(s)-1. All
                                % three of the *_3points_session_* logs above
                                % have a full 10 rounds (0-9) for points
                                % 3/10/14. Each plot's legend/title reports
                                % how many iterations were actually FOUND,
                                % which can be less than requested here.
POINT_IDS       = [3,10,14];   % run the whole analysis once per point in
                                  % this list (each point gets its own mean/
                                  % median-IQR/hex-map figure trio)

% ── Paths ────────────────────────────────────────────────────────────────
HERE        = fileparts(mfilename('fullpath'));
LOG_DIR     = fullfile(HERE, '..', 'logs');
RESULTS_DIR = fullfile(HERE, 'results');
if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end

SURFACE_NAMES  = {'flat', 'solid', 'hollow'};
SURFACE_FILES  = {FLAT_CSV, SOLID_CSV, HOLLOW_CSV};
% From SDU_color_palette_01 2.ase (parsed RGB swatches) -- picked the
% yellow/orange/brown trio, deliberately avoiding the green and salmon-red
% swatches from that same palette already used by the hex-map gradient
% (get_cmap.m), so the two plot types don't share colors with different
% meanings.
SURFACE_COLORS = [0.9490 0.7804 0.3608;   % flat   - SDU yellow  #F2C75C
                   0.8784 0.4941 0.2353;   % solid  - SDU orange  #E07E3C
                   0.4784 0.3765 0.2510];  % hollow - SDU brown   #7A6040

[point_ids, point_xy, own_cell_col] = muca_layout();
HEX_RADIUS = 8.0 / sqrt(3);

% ── Load ─────────────────────────────────────────────────────────────────
data = cell(1, 3);
fprintf('%s\n', repmat('=', 1, 70));
fprintf('  LOADED DATASETS\n');
fprintf('%s\n', repmat('=', 1, 70));
for s = 1:3
    csv_path = fullfile(LOG_DIR, SURFACE_FILES{s});
    fprintf('\n  %s : %s\n', upper(SURFACE_NAMES{s}), csv_path);
    if ~exist(csv_path, 'file')
        fprintf('    [warn] file not found -- skipping this surface\n');
        data{s} = [];
        continue;
    end
    d = read_muca_csv(csv_path);
    data{s} = d;

    press_mask = ismember(d.phase, {'press', 'hold', 'retract'});
    depths_here = unique(d.depth_mm(press_mask & ~isnan(d.depth_mm)));
    rounds_here = unique(d.round_idx(press_mask & d.round_idx >= 0));
    fprintf('    rows          : %d\n', d.n_rows);
    fprintf('    depths (mm)   : %s\n', mat2str(sort(depths_here)'));
    fprintf('    rounds        : %s\n', mat2str(sort(rounds_here)'));
    fprintf('    force zero    : %.4f N subtracted (ai0_to_force_N min over session)\n', d.force_zero_N);
end
fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('  Using DEPTH_MM = %g, POINT_IDS = %s\n', DEPTH_MM, mat2str(POINT_IDS));
fprintf('  NUM_ITERATIONS requested (flat/solid/hollow) = %s\n', mat2str(NUM_ITERATIONS));
fprintf('  (actual iterations found are reported per-plot below/on the legend)\n');
fprintf('%s\n', repmat('=', 1, 70));

for POINT_ID = POINT_IDS   %#ok<FXSET> -- run the full analysis once per point
fprintf('\n\n');
fprintf('%s\n', repmat('#', 1, 70));
fprintf('  POINT %d\n', POINT_ID);
fprintf('%s\n', repmat('#', 1, 70));

% ── 1) Force vs time (press-hold-retract), 3 surfaces overlaid ─────────────
% Two square, grid-free variants, both built on the same 0.1s-binned stats
% (muca_force_vs_time.m): (a) mean line only, (b) median line with a
% shaded Q1-Q3 band ("shadow plot").
force_series = cell(1, 3);   % cache per-surface t_bin/stats so both plots reuse one pass
fprintf('\nForce-vs-time iterations used (point %d, depth %g mm):\n', POINT_ID, DEPTH_MM);
for s = 1:3
    if isempty(data{s})
        force_series{s} = [];
        continue;
    end
    [t_rel, ~, t_bin, stats, n_used] = muca_force_vs_time( ...
        data{s}, DEPTH_MM, NUM_ITERATIONS(s), {'press', 'hold', 'retract'}, POINT_ID);
    fprintf('  %-8s: requested %d, found %d iteration(s)\n', ...
        SURFACE_NAMES{s}, NUM_ITERATIONS(s), n_used);
    if isempty(t_rel)
        fprintf('    [warn] no press/hold/retract rows -- skipping this surface\n');
        force_series{s} = [];
        continue;
    end
    force_series{s} = struct('t_bin', t_bin, 'stats', stats, 'n_used', n_used);
end

% Press/hold/retract phase timing is robot-commanded (ramp_s/hold_s), not
% surface-dependent, so average the boundary estimate across whichever
% surfaces have data for one shared background shading (muca_phase_boundaries.m).
pb = nan(3, 3);   % rows = surfaces, cols = [t_press_end, t_hold_end, t_total_end]
for s = 1:3
    if isempty(data{s})
        continue;
    end
    [pb(s, 1), pb(s, 2), pb(s, 3)] = muca_phase_boundaries(data{s}, DEPTH_MM, NUM_ITERATIONS(s), POINT_ID);
end
t_press_end = mean(pb(:, 1), 'omitnan');
t_hold_end  = mean(pb(:, 2), 'omitnan');
t_total_end = mean(pb(:, 3), 'omitnan');

% Fixed y-axis range per point (not data-driven): P03 tops out lower than
% P10/P14 since it's a corner point with fewer neighbors to load.
if POINT_ID == 3
    y_lim_max = 16;
    y_ticks   = [0 4 8 12 16];
else
    y_lim_max = 20;
    y_ticks   = [0 5 10 15 20];
end
X_LIM = [0, 10];   % fixed time window for all points/depths

FONT_NAME     = 'Helvetica';
AXIS_TITLE_FS = 12;   % xlabel/ylabel
TICK_FS       = 10;

% -- 1a) mean line only --------------------------------------------------
fig1a = figure('Position', [100 100 800 800]);
ax = axes('Parent', fig1a, 'Position', [0.15, 0.15, 0.70, 0.70]);   % equal width/height -- see note above on axis('square')
hold(ax, 'on');
xlim(ax, X_LIM);
ylim(ax, [0, y_lim_max]);
draw_phase_shading(ax, t_press_end, t_hold_end, t_total_end, y_lim_max);
legend_labels = {};
line_handles  = [];
for s = 1:3
    if isempty(force_series{s})
        continue;
    end
    fs = force_series{s};
    finite = ~isnan(fs.stats.mean);
    h = plot(ax, fs.t_bin(finite), fs.stats.mean(finite), '-', ...
        'Color', SURFACE_COLORS(s, :), 'LineWidth', 2.5);
    line_handles(end + 1) = h; %#ok<SAGROW>
    legend_labels{end + 1} = sprintf('%s (n=%d)', SURFACE_NAMES{s}, fs.n_used); %#ok<SAGROW>
end
xlabel(ax, 'Time (s)', 'FontName', FONT_NAME, 'FontSize', AXIS_TITLE_FS);
ylabel(ax, 'Force (N)', 'FontName', FONT_NAME, 'FontSize', AXIS_TITLE_FS);
set(ax, 'FontName', FONT_NAME, 'FontSize', TICK_FS, 'YTick', y_ticks, 'XTick', 0:2:10);
axis(ax, 'square');
pbaspect([1 1 1])
grid(ax, 'off');
box(ax, 'on');
if ~isempty(legend_labels)
    % Explicit LINE handles only (not the fill/patch objects in fig1b) so
    % every legend icon is a line, never a filled square. Legend placed
    % OUTSIDE the axes (above it) so it can never overlap the phase-shading
    % labels or data, regardless of this point's curve shapes; restore the
    % axes' own Position afterward since placing a legend outside an axes
    % auto-shrinks that axes to make room (same effect we saw with
    % colorbar() on the hex-map panels).
    orig_pos = get(ax, 'Position');
    legend(ax, line_handles, legend_labels, 'Location', 'northoutside', ...
        'FontName', FONT_NAME, 'FontSize', TICK_FS);
    set(ax, 'Position', orig_pos);
end
save_fig(fig1a, fullfile(RESULTS_DIR, sprintf('force_vs_time_mean_point%d_depth%gmm', POINT_ID, DEPTH_MM)));

% -- 1b) median line + Q1-Q3 shaded band ("shadow plot") -----------------
fig1b = figure('Position', [100 100 800 800]);
ax = axes('Parent', fig1b, 'Position', [0.15, 0.15, 0.70, 0.70]);   % equal width/height -- see note above on axis('square')
hold(ax, 'on');
xlim(ax, X_LIM);
ylim(ax, [0, y_lim_max]);
draw_phase_shading(ax, t_press_end, t_hold_end, t_total_end, y_lim_max);
legend_labels = {};
line_handles  = [];
for s = 1:3
    if isempty(force_series{s})
        continue;
    end
    fs = force_series{s};
    finite = ~isnan(fs.stats.median) & ~isnan(fs.stats.q1) & ~isnan(fs.stats.q3);
    tb  = fs.t_bin(finite);
    med = fs.stats.median(finite);
    q1  = fs.stats.q1(finite);
    q3  = fs.stats.q3(finite);
    fill(ax, [tb, fliplr(tb)], [q1, fliplr(q3)], SURFACE_COLORS(s, :), ...
        'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    h = plot(ax, tb, med, '-', 'Color', SURFACE_COLORS(s, :), 'LineWidth', 2.5);
    line_handles(end + 1) = h; %#ok<SAGROW>
    legend_labels{end + 1} = sprintf('%s (n=%d)', SURFACE_NAMES{s}, fs.n_used); %#ok<SAGROW>
end
xlabel(ax, 'Time (s)', 'FontName', FONT_NAME, 'FontSize', AXIS_TITLE_FS);
ylabel(ax, 'Force (N)', 'FontName', FONT_NAME, 'FontSize', AXIS_TITLE_FS);
set(ax, 'FontName', FONT_NAME, 'FontSize', TICK_FS, 'YTick', y_ticks, 'XTick', 0:2:10);
axis(ax, 'square');
grid(ax, 'off');
box(ax, 'on');
if ~isempty(legend_labels)
    orig_pos = get(ax, 'Position');
    legend(ax, line_handles, legend_labels, 'Location', 'northoutside', ...
        'FontName', FONT_NAME, 'FontSize', TICK_FS);
    set(ax, 'Position', orig_pos);
end
save_fig(fig1b, fullfile(RESULTS_DIR, sprintf('force_vs_time_median_iqr_point%d_depth%gmm', POINT_ID, DEPTH_MM)));

% -- 1c) all individual iterations (dotted) + median (solid) -------------
% Same styling as 1a/1b, but instead of a mean line or a shaded Q1-Q3 band,
% this shows every one of the (up to 10) raw iteration traces per surface
% as a thin dotted line, with the median drawn solid and bold on top --
% muca_force_vs_time_rounds.m keeps each round's trace separate rather
% than pooling them like muca_force_vs_time.m does.
fig1c = figure('Position', [100 100 800 800]);
ax = axes('Parent', fig1c, 'Position', [0.15, 0.15, 0.70, 0.70]);
hold(ax, 'on');
xlim(ax, X_LIM);
ylim(ax, [0, y_lim_max]);
draw_phase_shading(ax, t_press_end, t_hold_end, t_total_end, y_lim_max);
legend_labels = {};
line_handles  = [];
for s = 1:3
    if isempty(data{s})
        continue;
    end
    round_traces = muca_force_vs_time_rounds(data{s}, DEPTH_MM, NUM_ITERATIONS(s), ...
        {'press', 'hold', 'retract'}, POINT_ID);
    for r = 1:numel(round_traces)
        plot(ax, round_traces{r}.t, round_traces{r}.f, ':', ...
            'Color', SURFACE_COLORS(s, :), 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    if isempty(force_series{s})
        continue;
    end
    fs = force_series{s};
    finite = ~isnan(fs.stats.median);
    h = plot(ax, fs.t_bin(finite), fs.stats.median(finite), '-', ...
        'Color', SURFACE_COLORS(s, :), 'LineWidth', 2.5);
    line_handles(end + 1) = h; %#ok<SAGROW>
    legend_labels{end + 1} = sprintf('%s (n=%d)', SURFACE_NAMES{s}, fs.n_used); %#ok<SAGROW>
end
xlabel(ax, 'Time (s)', 'FontName', FONT_NAME, 'FontSize', AXIS_TITLE_FS);
ylabel(ax, 'Force (N)', 'FontName', FONT_NAME, 'FontSize', AXIS_TITLE_FS);
set(ax, 'FontName', FONT_NAME, 'FontSize', TICK_FS, 'YTick', y_ticks, 'XTick', 0:2:10);
axis(ax, 'square');
grid(ax, 'off');
box(ax, 'on');
if ~isempty(legend_labels)
    orig_pos = get(ax, 'Position');
    legend(ax, line_handles, legend_labels, 'Location', 'northoutside', ...
        'FontName', FONT_NAME, 'FontSize', TICK_FS);
    set(ax, 'Position', orig_pos);
end
save_fig(fig1c, fullfile(RESULTS_DIR, sprintf('force_vs_time_individual_point%d_depth%gmm', POINT_ID, DEPTH_MM)));

% ── 2) Hex schematics — full 19-cell response to pressing POINT_ID only,
%      3 surfaces, shared scale ────────────────────────────────────────────
% Unlike an own-cell map (each hexagon from its OWN press event), this is a
% single press event (POINT_ID only) read out across all 19 cells, so it
% shows cross-talk/spread from one indentation. The pressed point's hexagon
% is outlined in red.
hex_vals = nan(19, 3);
hex_n_used = zeros(1, 3);
fprintf('\nHex-map iterations used (point %d, depth %g mm):\n', POINT_ID, DEPTH_MM);
for s = 1:3
    if isempty(data{s})
        continue;
    end
    [hex_vals(:, s), hex_n_used(s)] = muca_point_response( ...
        data{s}, POINT_ID, DEPTH_MM, NUM_ITERATIONS(s), own_cell_col);
    fprintf('  %-8s: requested %d, found %d iteration(s)\n', ...
        SURFACE_NAMES{s}, NUM_ITERATIONS(s), hex_n_used(s));
end

all_vals = hex_vals(~isnan(hex_vals));
if isempty(all_vals)
    fprintf('  [warn] no hold-phase data for point %d at depth %g mm for any surface -- skipping hex plot\n', ...
        POINT_ID, DEPTH_MM);
else
    vmin = min(all_vals);
    vmax = max(all_vals);
    cmap = get_cmap();

    fig2 = figure('Position', [100 100 1500 550]);
    % Equal-size fixed positions for all 3 panels -- colorbar gets its own
    % reserved slot on the right so it doesn't shrink whichever axes it's
    % attached to (subplot()'s default auto-resize-on-colorbar behavior is
    % what made 'hollow' look smaller than 'flat'/'solid' before).
    panel_left  = [0.03, 0.35, 0.67];
    panel_width = 0.27;
    axes_list = cell(1, 3);
    for s = 1:3
        ax = axes('Parent', fig2, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
        axes_list{s} = ax;
        draw_hex_panel(ax, point_ids, point_xy, hex_vals(:, s), cmap, vmin, vmax, ...
            HEX_RADIUS, sprintf('%s (n=%d)', upper(SURFACE_NAMES{s}), hex_n_used(s)), POINT_ID);
    end

    colormap(fig2, cmap);
    for s = 1:3
        set(axes_list{s}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, 'normalized capacitive value (0-1)');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
    end

    fig_suptitle(fig2, sprintf( ...
        'Muca-Board -- full-cell response to pressing P%02d at %g mm', ...
        POINT_ID, DEPTH_MM));
    save_fig(fig2, fullfile(RESULTS_DIR, sprintf('hexmaps_point%d_depth%gmm', POINT_ID, DEPTH_MM)));
end

% ── 2b) Hex schematics, version 2 — median-of-iteration-means ──────────────
% Different aggregation from the plot above: instead of pooling every
% hold-phase row from every round into one big mean, average each round
% SEPARATELY first (one 19-value snapshot per iteration), then take the
% MEDIAN across the (up to NUM_ITERATIONS) per-round snapshots, per cell.
% More robust to a single outlier press than the pooled mean.
hex_vals_med = nan(19, 3);
hex_n_used_med = zeros(1, 3);
fprintf('\nHex-map (median-of-iterations) used (point %d, depth %g mm):\n', POINT_ID, DEPTH_MM);
for s = 1:3
    if isempty(data{s})
        continue;
    end
    [hex_vals_med(:, s), hex_n_used_med(s)] = muca_point_response_median( ...
        data{s}, POINT_ID, DEPTH_MM, NUM_ITERATIONS(s), own_cell_col);
    fprintf('  %-8s: requested %d, found %d iteration(s)\n', ...
        SURFACE_NAMES{s}, NUM_ITERATIONS(s), hex_n_used_med(s));
end

all_vals_med = hex_vals_med(~isnan(hex_vals_med));
if isempty(all_vals_med)
    fprintf('  [warn] no hold-phase data for point %d at depth %g mm for any surface -- skipping median hex plot\n', ...
        POINT_ID, DEPTH_MM);
else
    vmin = min(all_vals_med);
    vmax = max(all_vals_med);
    cmap = get_cmap();

    fig2b = figure('Position', [100 100 1500 550]);
    panel_left  = [0.03, 0.35, 0.67];
    panel_width = 0.27;
    axes_list = cell(1, 3);
    for s = 1:3
        ax = axes('Parent', fig2b, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
        axes_list{s} = ax;
        draw_hex_panel(ax, point_ids, point_xy, hex_vals_med(:, s), cmap, vmin, vmax, ...
            HEX_RADIUS, sprintf('%s (n=%d)', upper(SURFACE_NAMES{s}), hex_n_used_med(s)), POINT_ID);
    end

    colormap(fig2b, cmap);
    for s = 1:3
        set(axes_list{s}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, 'normalized capacitive value (0-1)');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
    end

    fig_suptitle(fig2b, sprintf( ...
        'Muca-Board -- full-cell response to pressing P%02d at %g mm (median of %d iterations)', ...
        POINT_ID, DEPTH_MM, max(NUM_ITERATIONS)));
    save_fig(fig2b, fullfile(RESULTS_DIR, sprintf('hexmaps_median_point%d_depth%gmm', POINT_ID, DEPTH_MM)));
end

end   % for POINT_ID = POINT_IDS

fprintf('\nFigures saved in: %s/\n', RESULTS_DIR);
