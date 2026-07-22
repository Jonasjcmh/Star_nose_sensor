%% plot_two_point_iterations.m
%
% Simple, editable MATLAB script -- everything happens top to bottom in
% this one file, no custom functions to jump to.
%
% For each of the 6 combinations of point (P08, P19) x surface (hollow,
% solid, flat) it makes TWO figures: a 2-row x 5-column grid.
%
%   Figure set 1 (plots_two_point_iterations/):
%       Row 1 = capacitance change  dC = Cp_pF - C0   (pF), offset removed
%       Row 2 = force module       |F - F0|            (N), offset removed
%   Figure set 2 (plots_two_point_iterations_deltaC_over_C0/), IN PARALLEL:
%       Row 1 = normalised capacitance change  dC/C0  (%)
%       Row 2 = force module       |F - F0|            (N)      -- same as above
%   Columns = the 5 indentation depths tested: 0, 1, 2, 3, 4 mm
%
% C0/F0 are the "no-load" baseline capacitance/force of that specific
% depth+iteration run, taken as the mean of the 'locate' phase (the robot
% has arrived but has not yet pressed into the surface -- confirmed flat
% in the raw data). Subtracting them removes the offset. Force is then
% taken as its absolute value (module) so it always reads as a positive
% response magnitude, regardless of push/pull sign.
%
% Axis limits and tick positions are set MANUALLY below (AXIS_* variables)
% -- nothing is auto-computed from the data. Edit those numbers directly
% to change what every figure shows; they are used for every dataset and
% every depth column alike. Font is Helvetica throughout.
%
% Each subplot overlays the 5 repeated iterations recorded at that depth,
% one color per iteration. Time is reset to 0 at the start of each
% iteration so all 5 lines in a subplot line up.
%
% To add/remove a dataset or point to a different file, just edit the
% DATASETS list below -- nothing else needs to change.

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

DEPTHS_MM = [0 1 2 3 4];     % one column per depth
ITERS     = [0 1 2 3 4];     % 5 repeats per depth
FONT_NAME = 'Helvetica';

% ================= MANUAL AXIS SCALE -- edit these directly =================
% Same limits/ticks are used for every dataset and every depth column.
DC_YLIM   = [-0.45  0.30];             % capacitance change dC (pF)
DC_YTICK  = [-0.45 -0.20  0.05  0.30];
PCT_YLIM  = [-25  20];                 % normalised change dC/C0 (%)
PCT_YTICK = [-25  -10    5    20];
F_YLIM    = [0  7];                    % force module |F - F0| (N)
F_YTICK   = [0  2.33  4.67  7];
T_XLIM    = [0  15];                   % time (s)
T_XTICK   = [0  5  10  15];
% ==============================================================================

% Force linearization from force_sensor_calibration/Matlab calibration/
% step1_loadcell_calibration.json (dataset v2): F [N] = LC_SLOPE * ai0 [V] + LC_OFFSET.
% Force here is recomputed straight from the raw ai0 voltage with this fit --
% the 'load_cell_N' column already in these CSVs was written with an older,
% different coefficient and is not used.
LC_SLOPE  = 4.113951054770791;    % N per V
LC_OFFSET = -19.28418747084478;   % N

% SDU brand colour palette (designguide.sdu.dk), one colour per iteration.
% Picked for maximum contrast between the 5 overlaid lines; RGB values
% straight from the guide, divided by 255 for MATLAB's [0-1] colour range.
SDU_RED    = [208  90  87] / 255;
SDU_GREEN  = [120 157  74] / 255;
SDU_ORANGE = [224 126  60] / 255;
SDU_BROWN  = [122  96  64] / 255;
SDU_YELLOW = [242 199  92] / 255;
ITER_COLORS = [SDU_RED; SDU_GREEN; SDU_ORANGE; SDU_BROWN; SDU_YELLOW];

% Turns a list of tick positions into short, readable labels (3 significant
% digits) instead of MATLAB's default long decimals -- e.g. 0.00604 instead
% of 0.0060381.
fmt_ticks = @(v) arrayfun(@(x) sprintf('%.3g', x), v, 'UniformOutput', false);

OUT_DIR      = fullfile(HERE, 'plots_two_point_iterations');
OUT_DIR_NORM = fullfile(HERE, 'plots_two_point_iterations_deltaC_over_C0');
if ~exist(OUT_DIR, 'dir');      mkdir(OUT_DIR);      end
if ~exist(OUT_DIR_NORM, 'dir'); mkdir(OUT_DIR_NORM); end

for d = 1:size(DATASETS, 1)

    label     = DATASETS{d, 1};
    fname     = DATASETS{d, 2};
    point_num = DATASETS{d, 3};

    csv_path = fullfile(LOG_DIR, fname);
    fprintf('%d/%d  reading %s (point %d)...\n', d, size(DATASETS, 1), fname, point_num);

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

    % keep only the rows for the point shown in this figure
    keep = point_col == point_num;
    depth_col = depth_col(keep);
    iter_col  = iter_col(keep);
    phase_col = phase_col(keep);
    ts_col    = ts_col(keep);
    cap_col   = cap_col(keep);
    ai0_col   = ai0_col(keep);

    % raw (signed) force from the current load-cell linearization, not the
    % CSV's own 'load_cell_N' column
    raw_force_col = LC_SLOPE * ai0_col + LC_OFFSET;

    % --- remove the offset: subtract each depth+iteration's own baseline
    % (mean during 'locate', i.e. before the surface is touched), then take
    % the force's absolute value so it reads as a positive response module ---
    dC_col    = nan(size(cap_col));
    pct_col   = nan(size(cap_col));
    force_col = nan(size(cap_col));
    t_col     = nan(size(cap_col));

    for c = 1:numel(DEPTHS_MM)
        for k = 1:numel(ITERS)
            rows = depth_col == DEPTHS_MM(c) & iter_col == ITERS(k);
            baseline_rows = rows & strcmp(phase_col, 'locate');
            C0 = mean(cap_col(baseline_rows));
            F0 = mean(raw_force_col(baseline_rows));

            dC_col(rows)    = cap_col(rows) - C0;
            pct_col(rows)   = 100 * (cap_col(rows) - C0) / C0;
            force_col(rows) = abs(raw_force_col(rows) - F0);
            t_col(rows)     = ts_col(rows) - min(ts_col(rows));   % iteration starts at t = 0
        end
    end

    %% ---- figure set 1: dC (pF) + force, one scale shared across this dataset's 5 depths ----
    fig1 = figure('Color', 'w', 'Position', [50 50 1700 600]);

    for c = 1:numel(DEPTHS_MM)
        depth_mm = DEPTHS_MM(c);
        cols_rows = depth_col == depth_mm;   % all 5 iterations at this depth

        % --- row 1: capacitance change, offset removed ---
        ax1 = subplot(2, numel(DEPTHS_MM), c);
        hold(ax1, 'on');
        for k = 1:numel(ITERS)
            rows = cols_rows & iter_col == ITERS(k);
            plot(ax1, t_col(rows), dC_col(rows), '-', 'Color', ITER_COLORS(k, :), 'LineWidth', 1.1);
        end
        title(ax1, sprintf('%.0f mm', depth_mm));

        ylim(ax1, DC_YLIM);
        set(ax1, 'YTick', DC_YTICK, 'YTickLabel', fmt_ticks(DC_YTICK));
        xlim(ax1, T_XLIM);
        set(ax1, 'XTick', T_XTICK, 'XTickLabel', fmt_ticks(T_XTICK));

        set(ax1, 'FontName', FONT_NAME, 'FontSize', 9, 'Box', 'off');
        if c == 1
            ylabel(ax1, 'dC = Cp - C0 (pF)');
            legend(ax1, {'iter 1', 'iter 2', 'iter 3', 'iter 4', 'iter 5'}, ...
                   'Location', 'best', 'FontSize', 7, 'FontName', FONT_NAME);
        end

        % --- row 2: force ---
        ax2 = subplot(2, numel(DEPTHS_MM), numel(DEPTHS_MM) + c);
        hold(ax2, 'on');
        for k = 1:numel(ITERS)
            rows = cols_rows & iter_col == ITERS(k);
            plot(ax2, t_col(rows), force_col(rows), '-', 'Color', ITER_COLORS(k, :), 'LineWidth', 1.1);
        end

        ylim(ax2, F_YLIM);
        set(ax2, 'YTick', F_YTICK, 'YTickLabel', fmt_ticks(F_YTICK));
        xlim(ax2, T_XLIM);
        set(ax2, 'XTick', T_XTICK, 'XTickLabel', fmt_ticks(T_XTICK));

        xlabel(ax2, 'time (s)');
        set(ax2, 'FontName', FONT_NAME, 'FontSize', 9, 'Box', 'off');
        if c == 1
            ylabel(ax2, '|F - F0| (N)');
        end
    end

    % Figure title on its own invisible axes (sgtitle is not available on
    % older MATLAB / GNU Octave, this works on both).
    title_ax = axes(fig1, 'Position', [0 0.96 1 0.04], 'Visible', 'off');
    text(title_ax, 0.5, 0.5, sprintf('Two-point iterations -- %s -- offset removed', strrep(label, '_', ' ')), ...
         'HorizontalAlignment', 'center', 'FontName', FONT_NAME, ...
         'FontSize', 13, 'FontWeight', 'bold');

    out_base = fullfile(OUT_DIR, label);
    print(fig1, [out_base '.png'], '-dpng', '-r150');
    print(fig1, [out_base '.svg'], '-dsvg');
    savefig(fig1, [out_base '.fig']);
    fprintf('saved -> %s.png / .svg / .fig\n', out_base);

    %% ---- figure set 2: dC/C0 (%) + force, IN PARALLEL, same local-scale rules ----
    fig2 = figure('Color', 'w', 'Position', [50 50 1700 600]);

    for c = 1:numel(DEPTHS_MM)
        depth_mm = DEPTHS_MM(c);
        cols_rows = depth_col == depth_mm;

        % --- row 1: normalised capacitance change dC/C0 ---
        ax1 = subplot(2, numel(DEPTHS_MM), c);
        hold(ax1, 'on');
        for k = 1:numel(ITERS)
            rows = cols_rows & iter_col == ITERS(k);
            plot(ax1, t_col(rows), pct_col(rows), '-', 'Color', ITER_COLORS(k, :), 'LineWidth', 1.1);
        end
        title(ax1, sprintf('%.0f mm', depth_mm));

        ylim(ax1, PCT_YLIM);
        set(ax1, 'YTick', PCT_YTICK, 'YTickLabel', fmt_ticks(PCT_YTICK));
        xlim(ax1, T_XLIM);
        set(ax1, 'XTick', T_XTICK, 'XTickLabel', fmt_ticks(T_XTICK));

        set(ax1, 'FontName', FONT_NAME, 'FontSize', 9, 'Box', 'off');
        if c == 1
            ylabel(ax1, 'dC / C0 (%)');
            legend(ax1, {'iter 1', 'iter 2', 'iter 3', 'iter 4', 'iter 5'}, ...
                   'Location', 'best', 'FontSize', 7, 'FontName', FONT_NAME);
        end

        % --- row 2: force (same data, same shared scale as figure set 1) ---
        ax2 = subplot(2, numel(DEPTHS_MM), numel(DEPTHS_MM) + c);
        hold(ax2, 'on');
        for k = 1:numel(ITERS)
            rows = cols_rows & iter_col == ITERS(k);
            plot(ax2, t_col(rows), force_col(rows), '-', 'Color', ITER_COLORS(k, :), 'LineWidth', 1.1);
        end

        ylim(ax2, F_YLIM);
        set(ax2, 'YTick', F_YTICK, 'YTickLabel', fmt_ticks(F_YTICK));
        xlim(ax2, T_XLIM);
        set(ax2, 'XTick', T_XTICK, 'XTickLabel', fmt_ticks(T_XTICK));

        xlabel(ax2, 'time (s)');
        set(ax2, 'FontName', FONT_NAME, 'FontSize', 9, 'Box', 'off');
        if c == 1
            ylabel(ax2, '|F - F0| (N)');
        end
    end

    title_ax = axes(fig2, 'Position', [0 0.96 1 0.04], 'Visible', 'off');
    text(title_ax, 0.5, 0.5, sprintf('Two-point iterations -- %s -- {\\Delta}C / C0', strrep(label, '_', ' ')), ...
         'HorizontalAlignment', 'center', 'FontName', FONT_NAME, ...
         'FontSize', 13, 'FontWeight', 'bold');

    out_base = fullfile(OUT_DIR_NORM, label);
    print(fig2, [out_base '.png'], '-dpng', '-r150');
    print(fig2, [out_base '.svg'], '-dsvg');
    savefig(fig2, [out_base '.fig']);
    fprintf('saved -> %s.png / .svg / .fig\n', out_base);
end

fprintf('\nDone. %d figures in %s\n%d figures in %s\n', ...
        size(DATASETS, 1), OUT_DIR, size(DATASETS, 1), OUT_DIR_NORM);
