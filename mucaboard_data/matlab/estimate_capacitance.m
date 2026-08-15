% estimate_capacitance.m — approximate mucaboard-reading <-> capacitance (pF)
% =========================================================================
% Empirically relates the mucaboard's normalized [0,1] cell reading to real
% capacitance (pF), by matching against a SEPARATE dataset where the same
% electrodes were connected directly to an LCR meter (Capacitance_measurement/
% logs) instead of the muca board -- the two were never measured
% simultaneously, so this is an INDIRECT link via (point, depth):
%
%   1. Flat & solid: the LCR sweep files use the SAME depth_mm labels
%      (5,6,7,8,9mm) as the mucaboard sessions, covering all 19 points --
%      matched directly, no offset.
%   2. Hollow: the LCR file's depth_mm (0-4mm) turns out to correspond to
%      the mucaboard session's depth_mm-5 (5-9mm) -- confirmed by
%      load_cell_N matching closely at that +5mm offset (see
%      match_lcr_muca_by_depth.m) -- and only covers points 3/10/14.
%
% CAVEAT: Cp_pF (direct LCR self-capacitance) and the muca board's
% normalized reading (the FT5x16 touch chip's own internal mutual-
% capacitance-derived signal) are DIFFERENT electrical quantities read via
% different topologies -- expect Cp_pF to DECREASE with press depth while
% the muca reading INCREASES (confirmed in this data). So the fit below is
% an empirical correlate for estimating pF from a muca reading, not a
% first-principles unit conversion.
%
% Usage
% -----
%   octave-cli estimate_capacitance.m
%   (or open/run in MATLAB)

clear; close all; clc;

% ── CONFIG ───────────────────────────────────────────────────────────────
HERE         = fileparts(mfilename('fullpath'));
MUCA_LOG_DIR = fullfile(HERE, '..', 'logs');
CAP_LOG_DIR  = fullfile(HERE, '..', '..', 'Capacitance_measurement', 'logs');
RESULTS_DIR  = fullfile(HERE, '..', 'capacitive_relationship');
if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end

NUM_ITERATIONS_MUCA = 5;   % round_idx 0..4, matches the *_5iter(s)_* mucaboard sessions

SURFACE_NAMES = {'flat', 'solid', 'hollow'};
SURFACE_COLORS = [0.9490 0.7804 0.3608;   % flat   - SDU yellow
                   0.8784 0.4941 0.2353;   % solid  - SDU orange
                   0.4784 0.3765 0.2510];  % hollow - SDU brown

MUCA_CSV = { ...
    fullfile(MUCA_LOG_DIR, 'flat_new_5iters_session_20260813_195924.csv'), ...
    fullfile(MUCA_LOG_DIR, 'long_dataset_solid_dome_session_20260812_192705.csv'), ...
    fullfile(MUCA_LOG_DIR, 'hollow_dome_5iters_session_20260814_132756.csv') ...
};

LCR_FILES = { ...
    { fullfile(CAP_LOG_DIR, 'ramp_collector_20260714_163437_flat_0.csv'), ...
      fullfile(CAP_LOG_DIR, 'ramp_collector_20260714_151051_flat_1.csv'), ...
      fullfile(CAP_LOG_DIR, 'ramp_collector_20260714_135538_flat_2.csv'), ...
      fullfile(CAP_LOG_DIR, 'ramp_collector_20260714_124706_flat_3.csv'), ...
      fullfile(CAP_LOG_DIR, 'ramp_collector_20260714_113352_flat_4.csv') }, ...
    { fullfile(CAP_LOG_DIR, 'ramp_collector_20260710_164240_solidD_0mm.csv'), ...
      fullfile(CAP_LOG_DIR, 'ramp_collector_20260710_152938_solidD_1mm.csv'), ...
      fullfile(CAP_LOG_DIR, 'ramp_collector_20260709_161711_solidD_2mm.csv'), ...
      fullfile(CAP_LOG_DIR, 'ramp_collector_20260710_140020_solidD_3mm.csv'), ...
      fullfile(CAP_LOG_DIR, 'ramp_collector_20260709_144320_solidD_4mm.csv') }, ...
    { fullfile(CAP_LOG_DIR, 'two_point_iterations_P10_P03_P14_20260804_171210_hollow.csv') } ...
};

DEPTH_OFFSET = [0, 0, 5];   % added to LCR depth_mm before matching mucaboard depth_mm

[point_ids, point_xy, own_cell_col] = muca_layout();
HEX_RADIUS = 8.0 / sqrt(3);

% ── Load, match, fit — per surface ──────────────────────────────────────
fits = cell(1, 3);
matched_all = cell(1, 3);
lcr_rows_all = cell(1, 3);
muca_rows_all = cell(1, 3);
perpoint_fits = cell(1, 3);
delta_c_pf_all = nan(19, 3);
delta_c_all = nan(19, 3);
delta_v_all = nan(19, 3);
MUCA_DEPTH_SHALLOW = 5;
MUCA_DEPTH_DEEP    = 10;

fprintf('%s\n', repmat('=', 1, 70));
fprintf('  CAPACITANCE ESTIMATION -- muca reading vs LCR-measured Cp (pF)\n');
fprintf('%s\n', repmat('=', 1, 70));

for s = 1:3
    fprintf('\n-- %s --\n', upper(SURFACE_NAMES{s}));

    lcr_d = load_lcr_files(LCR_FILES{s});
    lcr_rows = lcr_stats_by_point_depth(lcr_d);
    lcr_rows_all{s} = lcr_rows;
    fprintf('  LCR   : %d rows loaded from %d file(s), %d (point,depth) groups\n', ...
        lcr_d.n_rows, numel(LCR_FILES{s}), size(lcr_rows, 1));

    [delta_c_pf_all(:, s), delta_c_all(:, s)] = lcr_delta_c_over_c0(lcr_rows);

    muca_d = read_muca_csv(MUCA_CSV{s});
    muca_rows = muca_stats_by_point_depth(muca_d, own_cell_col, NUM_ITERATIONS_MUCA);
    muca_rows_all{s} = muca_rows;
    fprintf('  MUCA  : %d rows loaded, %d (point,depth) groups\n', muca_d.n_rows, size(muca_rows, 1));

    [delta_v_all(:, s), v0_skipped] = muca_delta_v_over_v0(muca_rows, MUCA_DEPTH_SHALLOW, MUCA_DEPTH_DEEP);
    for i = 1:size(v0_skipped, 1)
        fprintf('  [warn] point %d: V0=%.4f at %dmm is below the noise-floor threshold -- excluded from delta-V/V0 (ratio would be unstable)\n', ...
            v0_skipped(i, 1), v0_skipped(i, 2), MUCA_DEPTH_SHALLOW);
    end

    matched = match_lcr_muca_by_depth(lcr_rows, muca_rows, DEPTH_OFFSET(s));
    matched_all{s} = matched;
    fprintf('  MATCH : %d (point,depth) pairs matched (offset=+%dmm)\n', size(matched, 1), DEPTH_OFFSET(s));

    if size(matched, 1) < 2
        fprintf('  [warn] fewer than 2 matched points -- skipping fit\n');
        fits{s} = [];
        continue;
    end

    x = matched(:, 6);   % muca normalized value
    y = matched(:, 5);   % Cp_pF
    p = polyfit(x, y, 1);
    yfit = polyval(p, x);
    resid = y - yfit;
    ss_res = sum(resid .^ 2);
    ss_tot = sum((y - mean(y)) .^ 2);
    r2 = 1 - ss_res / ss_tot;
    rmse = sqrt(mean(resid .^ 2));
    fits{s} = struct('slope', p(1), 'intercept', p(2), 'r2', r2, 'rmse', rmse, 'n', numel(x));

    fprintf('  FIT (pooled, all points) : Cp_pF = %.4f * muca_value + %.4f   (R2=%.3f, RMSE=%.4f pF, n=%d)\n', ...
        p(1), p(2), r2, rmse, numel(x));

    pp = fit_by_point(matched);
    perpoint_fits{s} = pp;
    fprintf('  FIT (per point):\n');
    fprintf('    %6s %8s %10s %10s %6s\n', 'point', 'slope', 'intercept', 'R2', 'n');
    for i = 1:size(pp, 1)
        fprintf('    %6d %8.3f %10.3f %6.3f %6d\n', pp(i, 1), pp(i, 2), pp(i, 3), pp(i, 4), pp(i, 5));
    end
    if ~isempty(pp)
        fprintf('    mean per-point R2 = %.3f (pooled R2 = %.3f)\n', mean(pp(:, 4)), r2);
    end

    fprintf('  %6s %8s %10s %10s %10s %10s\n', 'point', 'depth', 'lcr_F(N)', 'muca_F(N)', 'Cp(pF)', 'muca_val');
    for i = 1:size(matched, 1)
        fprintf('  %6d %8.1f %10.3f %10.3f %10.4f %10.4f\n', matched(i, 1), matched(i, 2), ...
            matched(i, 3), matched(i, 4), matched(i, 5), matched(i, 6));
    end
end

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('  MUCA-ONLY delta-V/V0 (depth %dmm -> %dmm), own normalized reading\n', ...
    MUCA_DEPTH_SHALLOW, MUCA_DEPTH_DEEP);
fprintf('%s\n', repmat('=', 1, 70));
fprintf('  %6s %10s %10s %10s\n', 'point', 'flat', 'solid', 'hollow');
for p = 1:19
    fprintf('  %6d %10.4f %10.4f %10.4f\n', p, delta_v_all(p, 1), delta_v_all(p, 2), delta_v_all(p, 3));
end

% ── Plot: 3 square panels, scatter + fit line, one per surface ─────────────
FONT_NAME = 'Helvetica';

fig = figure('Position', [100 100 1900 700]);
panel_left  = [0.06, 0.38, 0.70];
panel_width = 0.27;

for s = 1:3
    ax = axes('Parent', fig, 'Position', [panel_left(s), 0.15, panel_width, 0.70]);
    hold(ax, 'on');
    box(ax, 'on');
    grid(ax, 'on');

    if ~isempty(matched_all{s}) && ~isempty(fits{s})
        x = matched_all{s}(:, 6);
        y = matched_all{s}(:, 5);
        scatter(ax, x, y, 45, SURFACE_COLORS(s, :), 'filled');
        xline = linspace(min(x), max(x), 50);
        yline = polyval([fits{s}.slope, fits{s}.intercept], xline);
        plot(ax, xline, yline, '-', 'Color', SURFACE_COLORS(s, :) * 0.6, 'LineWidth', 2.0);

        eq_str1 = sprintf('Cp = %.3f*v + %.3f', fits{s}.slope, fits{s}.intercept);
        eq_str2 = sprintf('R2 = %.3f, n=%d', fits{s}.r2, fits{s}.n);
        text(ax, 0.05, 0.95, eq_str1, 'Units', 'normalized', 'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'top', 'FontName', FONT_NAME, 'FontSize', 9, 'Interpreter', 'none');
        text(ax, 0.05, 0.88, eq_str2, 'Units', 'normalized', 'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'top', 'FontName', FONT_NAME, 'FontSize', 9, 'Interpreter', 'none');
    end

    xlabel(ax, 'Muca normalized reading (0-1)', 'FontName', FONT_NAME, 'FontSize', 12);
    ylabel(ax, 'Estimated capacitance (pF)', 'FontName', FONT_NAME, 'FontSize', 12);
    title(ax, upper(SURFACE_NAMES{s}), 'FontName', FONT_NAME, 'FontSize', 12, 'FontWeight', 'bold');
    set(ax, 'FontName', FONT_NAME, 'FontSize', 10);
    axis(ax, 'square');
end

fig_suptitle(fig, 'Muca-Board -- estimated capacitance (pF) vs normalized reading, per surface');
save_fig(fig, fullfile(RESULTS_DIR, 'capacitance_estimation'));

% ── Per-point linear relations — one figure per surface, small-multiples ──
% Pooling all 19 points into a single fit (the plot above) mixes together
% pads with very different absolute capacitance baselines and different
% depth-sensitivity, which is why its R^2 is much lower than most
% individual pads' own fits (see fit_by_point.m / console FIT per point).
GRID_ROWS = 4;
GRID_COLS = 5;

for s = 1:3
    if isempty(perpoint_fits{s}) || isempty(matched_all{s})
        continue;
    end
    pp = perpoint_fits{s};
    matched = matched_all{s};

    figp = figure('Position', [100 100 1700 1300]);
    for i = 1:size(pp, 1)
        pt = pp(i, 1);
        row_i = ceil(i / GRID_COLS);
        col_i = mod(i - 1, GRID_COLS) + 1;
        left = 0.03 + (col_i - 1) * (0.94 / GRID_COLS);
        bottom = 1 - 0.06 - row_i * (0.90 / GRID_ROWS);
        ax = axes('Parent', figp, 'Position', [left, bottom, 0.94 / GRID_COLS * 0.82, 0.90 / GRID_ROWS * 0.72]);
        hold(ax, 'on');
        box(ax, 'on');

        sub = matched(matched(:, 1) == pt, :);
        x = sub(:, 6);
        y = sub(:, 5);
        scatter(ax, x, y, 20, SURFACE_COLORS(s, :), 'filled');
        if numel(x) >= 2
            xline = linspace(min(x), max(x), 20);
            yline = polyval([pp(i, 2), pp(i, 3)], xline);
            plot(ax, xline, yline, '-', 'Color', SURFACE_COLORS(s, :) * 0.6, 'LineWidth', 1.5);
        end
        title(ax, sprintf('P%02d  R2=%.2f', pt, pp(i, 4)), 'FontName', FONT_NAME, 'FontSize', 8);
        set(ax, 'FontName', FONT_NAME, 'FontSize', 7);
    end
    fig_suptitle(figp, sprintf('Muca-Board -- per-point Cp(pF) vs muca reading, %s', upper(SURFACE_NAMES{s})));
    save_fig(figp, fullfile(RESULTS_DIR, sprintf('capacitance_perpoint_%s', SURFACE_NAMES{s})));
end

% ── Hex maps — absolute capacitance swing delta-C, per point (picoFarads) ──
% delta-C = max(Cp) - min(Cp) across the LCR depth sweep, per point
% (lcr_delta_c_over_c0.m) -- the raw pF swing, NOT normalized by baseline.
% Useful to see which pads have the largest absolute signal in real units,
% as opposed to the relative delta-C/C0 map below.
all_delta_pf = delta_c_pf_all(~isnan(delta_c_pf_all));
if isempty(all_delta_pf)
    fprintf('  [warn] no delta-C (pF) values available -- skipping hex map\n');
else
    vmin = min(all_delta_pf);
    vmax = max(all_delta_pf);
    cmap = get_cmap();

    fig1b = figure('Position', [100 100 1500 550]);
    panel_left1b  = [0.03, 0.35, 0.67];
    panel_width1b = 0.27;
    axes_list = cell(1, 3);
    for s = 1:3
        ax = axes('Parent', fig1b, 'Position', [panel_left1b(s), 0.12, panel_width1b, 0.72]);
        axes_list{s} = ax;
        draw_hex_panel(ax, point_ids, point_xy, delta_c_pf_all(:, s), cmap, vmin, vmax, ...
            HEX_RADIUS, upper(SURFACE_NAMES{s}));
    end
    colormap(fig1b, cmap);
    for s = 1:3
        set(axes_list{s}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, 'delta-C (pF)');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left1b(s), 0.12, panel_width1b, 0.72]);
    end
    fig_suptitle(fig1b, 'Muca-Board -- absolute capacitance swing delta-C (pF), per point (LCR-measured)');
    save_fig(fig1b, fullfile(RESULTS_DIR, 'capacitance_deltaC_pF_hexmap'));
end

% ── Hex maps — relative capacitance change delta-C/C0 per point ───────────
% delta-C/C0 = (max(Cp) - min(Cp)) / min(Cp) across the LCR depth sweep, per
% point (lcr_delta_c_over_c0.m), where C0 = min(Cp) is the LOWEST
% capacitance value observed for that point on that surface -- a standard
% capacitive-sensing normalization that corrects for each pad's very
% different absolute baseline, unlike the raw hex maps in analyze_mucaboard.m.
all_delta = delta_c_all(~isnan(delta_c_all));
if isempty(all_delta)
    fprintf('  [warn] no delta-C/C0 values available -- skipping hex map\n');
else
    vmin = min(all_delta);
    vmax = max(all_delta);
    cmap = get_cmap();

    fig2 = figure('Position', [100 100 1500 550]);
    panel_left2  = [0.03, 0.35, 0.67];
    panel_width2 = 0.27;
    axes_list = cell(1, 3);
    for s = 1:3
        ax = axes('Parent', fig2, 'Position', [panel_left2(s), 0.12, panel_width2, 0.72]);
        axes_list{s} = ax;
        draw_hex_panel(ax, point_ids, point_xy, delta_c_all(:, s), cmap, vmin, vmax, ...
            HEX_RADIUS, upper(SURFACE_NAMES{s}));
    end
    colormap(fig2, cmap);
    for s = 1:3
        set(axes_list{s}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, 'delta-C / C0 (fraction)');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left2(s), 0.12, panel_width2, 0.72]);
    end
    fig_suptitle(fig2, 'Muca-Board -- relative capacitance change delta-C/C0, per point (LCR-measured)');
    save_fig(fig2, fullfile(RESULTS_DIR, 'capacitance_deltaC_hexmap'));
end

% ── Hex maps — delta-V/V0 from the MUCA BOARD'S OWN reading (no LCR) ──────
% Same delta-over-baseline idea as the LCR-based map above, but computed
% entirely from the mucaboard's own normalized [0,1] signal (depth
% MUCA_DEPTH_SHALLOW -> MUCA_DEPTH_DEEP), not the LCR's Cp_pF. This gives
% full 19-point coverage on ALL THREE surfaces -- the LCR-based map above is
% stuck at 3 points for hollow, since that's all the direct-LCR-connected
% hollow dataset covers.
all_delta_v = delta_v_all(~isnan(delta_v_all));
if isempty(all_delta_v)
    fprintf('  [warn] no delta-V/V0 values available -- skipping muca-only hex map\n');
else
    vmin = min(all_delta_v);
    vmax = max(all_delta_v);
    cmap = get_cmap();

    fig3 = figure('Position', [100 100 1500 550]);
    panel_left3  = [0.03, 0.35, 0.67];
    panel_width3 = 0.27;
    axes_list = cell(1, 3);
    for s = 1:3
        ax = axes('Parent', fig3, 'Position', [panel_left3(s), 0.12, panel_width3, 0.72]);
        axes_list{s} = ax;
        draw_hex_panel(ax, point_ids, point_xy, delta_v_all(:, s), cmap, vmin, vmax, ...
            HEX_RADIUS, upper(SURFACE_NAMES{s}));
    end
    colormap(fig3, cmap);
    for s = 1:3
        set(axes_list{s}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, 'delta-V / V0 (fraction)');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left3(s), 0.12, panel_width3, 0.72]);
    end
    fig_suptitle(fig3, sprintf( ...
        'Muca-Board -- relative reading change delta-V/V0, per point (muca-only, %dmm -> %dmm)', ...
        MUCA_DEPTH_SHALLOW, MUCA_DEPTH_DEEP));
    save_fig(fig3, fullfile(RESULTS_DIR, 'muca_deltaV_hexmap'));
end

fprintf('\nFigures saved in: %s/\n', RESULTS_DIR);
