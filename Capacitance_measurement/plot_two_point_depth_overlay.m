%% plot_two_point_depth_overlay.m
%
% Companion to plot_two_point_iterations.m. Instead of one column per
% depth, this script overlays all 5 depths in ONE square panel -- one
% line per depth -- so the growth of the response with depth is visible
% directly, at a glance, in a single plot.
%
% Only ONE iteration per dataset is used (pick which one in ITER_TO_SHOW
% below -- 0 to 4). The capacitance channel (dC/C0) is run through a
% Butterworth low-pass filter first (order + cutoff frequency editable in
% FILTER_ORDER/FILTER_CUTOFF_HZ below); force is plotted raw (module,
% offset removed, same definition as plot_two_point_iterations.m).
%
% For each of the 6 point+surface datasets, saves ONE figure with two
% square panels:
%   Left  = filtered dC/C0 (%), 5 depths overlaid
%   Right = force module |F - F0| (N), 5 depths overlaid
%
% Same philosophy as plot_two_point_iterations.m: everything is in this
% one file, no custom functions, axis limits/ticks/iteration choice are
% all manual tables you edit directly.

clear; clc; close all;

HERE = fileparts(mfilename('fullpath'));
LOG_DIR = fullfile(HERE, 'logs');

% Each row: {label for titles/filenames, CSV filename in logs/, point number to keep}
DATASETS = {
    'P08_hollow', 'two_point_iterations_P08_P19_20260720_122906_hollow.csv', 8
    'P19_hollow', 'two_point_iterations_P08_P19_20260720_122906_hollow.csv', 19
    'P08_solid',  'two_point_iterations_P08_P19_20260720_133940_solid.csv',  8
    'P19_solid',  'two_point_iterations_P08_P19_20260720_133940_solid.csv',  19
    'P08_flat',   'two_point_iterations_P08_20260720_141402_flat.csv',      8
    'P19_flat',   'two_point_iterations_P19_20260720_140639_flat.csv',      19
};

DEPTHS_MM = [0 1 2 3 4];     % one overlaid line per depth
ITERS     = [0 1 2 3 4];     % the 5 recorded repeats -- pick ONE below
FONT_NAME = 'Helvetica';
TITLE_FONT_SIZE = 12;
TICK_FONT_SIZE  = 10;

% ================= WHICH ITERATION TO SHOW -- edit per dataset =============
% One row per dataset (same order as DATASETS above). Pick the iteration
% index (0-4) whose 5-depth overlay you want to see for that dataset.
ITER_TO_SHOW = {
    'P08_hollow', 0
    'P19_hollow', 0
    'P08_solid',  0
    'P19_solid',  0
    'P08_flat',   0
    'P19_flat',   0
};
% ==============================================================================

% ================= BUTTERWORTH FILTER -- edit these directly ================
% Applied to the dC/C0 (%) trace only, one depth at a time, with filtfilt
% (zero-phase -- no time shift). Fs is measured from each dataset's own
% timestamps (~99 Hz for this rig), so cutoff is set in Hz, not normalised.
FILTER_ORDER     = 4;    % filter order N, i.e. butter(N, Wn)
FILTER_CUTOFF_HZ = 5;    % low-pass cutoff frequency (Hz)
% ==============================================================================

% ================= MANUAL AXIS LIMITS -- edit these directly ================
% One row per dataset (same order as DATASETS above).
%                label         PCT_YLIM (%)   F_YLIM (N)   T_XLIM (s)
OVERLAY_YLIM = {
    'P08_hollow', [-3    0.5],  [0  2.5],    [0  15]
    'P19_hollow', [-3.2  0.5],  [0  2.4],    [0  15]
    'P08_solid',  [-5    12],   [0  5.0],    [0  15]
    'P19_solid',  [-25   17],   [0  6.6],    [0  15]
    'P08_flat',   [-18.5 17],   [0  5.2],    [0  15]
    'P19_flat',   [-5.5  18],   [0  6.2],    [0  15]
};
% ==============================================================================

% ================= MANUAL AXIS TICKS -- edit these directly =================
% One row per dataset. Type exactly the numbers you want to see.
%                label         PCT_YTICK (%)          F_YTICK (N)      T_XTICK (s)
OVERLAY_YTICK = {
    'P08_hollow', [-3 -1.5 0 0.5],       [0 1 2 2.5],     [0 5 10 15]
    'P19_hollow', [-3.2 -1.5 0 0.5],     [0 1 2 2.4],     [0 5 10 15]
    'P08_solid',  [-5 0 6 12],           [0 2.5 5],       [0 5 10 15]
    'P19_solid',  [-25 -12 0 17],        [0 3.3 6.6],     [0 5 10 15]
    'P08_flat',   [-18.5 -8 0 17],       [0 2.6 5.2],     [0 5 10 15]
    'P19_flat',   [-5.5 0 9 18],         [0 3.1 6.2],     [0 5 10 15]
};
% ==============================================================================

% Force linearization from force_sensor_calibration/Matlab calibration/
% step1_loadcell_calibration.json (dataset v2): F [N] = LC_SLOPE * ai0 [V] + LC_OFFSET.
LC_SLOPE  = 4.113951054770791;    % N per V
LC_OFFSET = -19.28418747084478;   % N

% SDU brand colour palette (designguide.sdu.dk), one colour per DEPTH here
% (plot_two_point_iterations.m uses the same 5 colours per ITERATION instead).
SDU_RED    = [208  90  87] / 255;
SDU_GREEN  = [120 157  74] / 255;
SDU_ORANGE = [224 126  60] / 255;
SDU_BROWN  = [122  96  64] / 255;
SDU_YELLOW = [242 199  92] / 255;
DEPTH_COLORS = [SDU_RED; SDU_GREEN; SDU_ORANGE; SDU_BROWN; SDU_YELLOW];

% Same tick-label formatter as plot_two_point_iterations.m
fmt_ticks = @(v) arrayfun(@(x) sprintf('%.3g', x), v, 'UniformOutput', false);

OUT_DIR = fullfile(HERE, 'plots_two_point_depth_overlay');
if ~exist(OUT_DIR, 'dir'); mkdir(OUT_DIR); end

for d = 1:size(DATASETS, 1)

    label       = DATASETS{d, 1};
    fname       = DATASETS{d, 2};
    point_num   = DATASETS{d, 3};
    iter_to_show = ITER_TO_SHOW{d, 2};

    csv_path = fullfile(LOG_DIR, fname);
    fprintf('%d/%d  reading %s (point %d, iteration %d)...\n', ...
            d, size(DATASETS, 1), fname, point_num, iter_to_show);

    %% ---- read the CSV (plain text scan -- no custom functions) ----
    fid = fopen(csv_path, 'r');
    header_line = fgetl(fid);
    col_names = strsplit(header_line, ',');
    fmt = repmat({'%f'}, 1, numel(col_names));
    fmt{strcmp(col_names, 'datetime')} = '%s';   % non-numeric columns
    fmt{strcmp(col_names, 'phase')}    = '%s';
    raw = textscan(fid, strjoin(fmt, ''), 'Delimiter', ',');
    fclose(fid);

    point_col = raw{strcmp(col_names, 'point')};
    depth_col = raw{strcmp(col_names, 'depth_mm')};
    iter_col  = raw{strcmp(col_names, 'iter_idx')};
    phase_col = raw{strcmp(col_names, 'phase')};
    ts_col    = raw{strcmp(col_names, 'timestamp')};
    cap_col   = raw{strcmp(col_names, 'Cp_pF')};
    ai0_col   = raw{strcmp(col_names, 'ai0')};

    % keep only the rows for the point AND the one iteration shown in this figure
    keep = point_col == point_num & iter_col == iter_to_show;
    depth_col = depth_col(keep);
    phase_col = phase_col(keep);
    ts_col    = ts_col(keep);
    cap_col   = cap_col(keep);
    ai0_col   = ai0_col(keep);

    % raw (signed) force from the current load-cell linearization, not the
    % CSV's own 'load_cell_N' column
    raw_force_col = LC_SLOPE * ai0_col + LC_OFFSET;

    % measure this recording's sample rate from its own timestamps, then
    % turn FILTER_CUTOFF_HZ into the normalised cutoff butter() needs
    dt = median(diff(ts_col));
    Fs = 1 / dt;
    Wn = FILTER_CUTOFF_HZ / (Fs / 2);
    [b, a] = butter(FILTER_ORDER, Wn);

    %% ---- one square-panel figure per dataset: filtered dC/C0 | force, 5 depths overlaid ----
    fig = figure('Color', 'w', 'Position', [50 50 1100 550]);

    % --- left panel: filtered dC/C0 (%), 5 depths overlaid ---
    ax1 = subplot(1, 2, 1);
    hold(ax1, 'on');
    for c = 1:numel(DEPTHS_MM)
        rows = depth_col == DEPTHS_MM(c);
        baseline_rows = rows & strcmp(phase_col, 'locate');
        C0 = mean(cap_col(baseline_rows));

        t   = ts_col(rows) - min(ts_col(rows));   % this depth's run starts at t = 0
        pct = 100 * (cap_col(rows) - C0) / C0;
        pct_filt = filtfilt(b, a, pct);

        plot(ax1, t, pct_filt, '-', 'Color', DEPTH_COLORS(c, :), 'LineWidth', 1.4);
    end
    title(ax1, 'dC / C0, filtered (5 depths overlaid)', 'FontName', FONT_NAME, 'FontSize', TITLE_FONT_SIZE);
    ylabel(ax1, 'dC / C0 (%)', 'FontName', FONT_NAME, 'FontSize', TITLE_FONT_SIZE);
    xlabel(ax1, 'time (s)', 'FontName', FONT_NAME, 'FontSize', TITLE_FONT_SIZE);
    ylim(ax1, OVERLAY_YLIM{d, 2});
    set(ax1, 'YTick', OVERLAY_YTICK{d, 2}, 'YTickLabel', fmt_ticks(OVERLAY_YTICK{d, 2}));
    xlim(ax1, OVERLAY_YLIM{d, 4});
    set(ax1, 'XTick', OVERLAY_YTICK{d, 4}, 'XTickLabel', fmt_ticks(OVERLAY_YTICK{d, 4}));
    set(ax1, 'FontName', FONT_NAME, 'FontSize', TICK_FONT_SIZE, 'Box', 'off');
    axis(ax1, 'square');
    legend(ax1, {'0 mm', '1 mm', '2 mm', '3 mm', '4 mm'}, ...
           'Location', 'best', 'FontSize', 7, 'FontName', FONT_NAME);

    % --- right panel: force module |F - F0| (N), 5 depths overlaid ---
    ax2 = subplot(1, 2, 2);
    hold(ax2, 'on');
    for c = 1:numel(DEPTHS_MM)
        rows = depth_col == DEPTHS_MM(c);
        baseline_rows = rows & strcmp(phase_col, 'locate');
        F0 = mean(raw_force_col(baseline_rows));

        t = ts_col(rows) - min(ts_col(rows));
        force = abs(raw_force_col(rows) - F0);

        plot(ax2, t, force, '-', 'Color', DEPTH_COLORS(c, :), 'LineWidth', 1.4);
    end
    title(ax2, 'force module (5 depths overlaid)', 'FontName', FONT_NAME, 'FontSize', TITLE_FONT_SIZE);
    ylabel(ax2, '|F - F0| (N)', 'FontName', FONT_NAME, 'FontSize', TITLE_FONT_SIZE);
    xlabel(ax2, 'time (s)', 'FontName', FONT_NAME, 'FontSize', TITLE_FONT_SIZE);
    ylim(ax2, OVERLAY_YLIM{d, 3});
    set(ax2, 'YTick', OVERLAY_YTICK{d, 3}, 'YTickLabel', fmt_ticks(OVERLAY_YTICK{d, 3}));
    xlim(ax2, OVERLAY_YLIM{d, 4});
    set(ax2, 'XTick', OVERLAY_YTICK{d, 4}, 'XTickLabel', fmt_ticks(OVERLAY_YTICK{d, 4}));
    set(ax2, 'FontName', FONT_NAME, 'FontSize', TICK_FONT_SIZE, 'Box', 'off');
    axis(ax2, 'square');
    legend(ax2, {'0 mm', '1 mm', '2 mm', '3 mm', '4 mm'}, ...
           'Location', 'best', 'FontSize', 7, 'FontName', FONT_NAME);

    title_ax = axes(fig, 'Position', [0 0.96 1 0.04], 'Visible', 'off');
    text(title_ax, 0.5, 0.5, sprintf('Two-point depth overlay -- %s -- iteration %d', ...
         strrep(label, '_', ' '), iter_to_show), ...
         'HorizontalAlignment', 'center', 'FontName', FONT_NAME, ...
         'FontSize', 13, 'FontWeight', 'bold');

    out_base = fullfile(OUT_DIR, label);
    print(fig, [out_base '.png'], '-dpng', '-r150');
    print(fig, [out_base '.svg'], '-dsvg');
    savefig(fig, [out_base '.fig']);
    fprintf('saved -> %s.png / .svg / .fig\n', out_base);
end

fprintf('\nDone. %d figures in %s\n', size(DATASETS, 1), OUT_DIR);
