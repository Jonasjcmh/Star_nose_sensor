% =========================================================================
% capacitance_19points_analysis.m
%
% Same cross-talk hex-map analysis as simple_19points_analysis.m (same
% data loading, same TRUE_TO_CODE board-label correction, same identity
% own_cell_col, same per-iteration-mean-then-median-across-iterations
% statistics), but the final plots are in REAL CAPACITANCE (pF) instead
% of raw counts / normalized [0,1] -- using the per-cell, per-surface
% linear relationship already derived and verified in
% Capacitance_calibration_Muca_LCR_Futek/results/muca_v_to_cp_table.csv:
%
%   Cp_pF = A(cell, surface) * V_own + B(cell, surface)
%
% (see that folder's chained_muca_v_to_cp_via_force.m for how A/B were
% obtained: mucaboard_data_raw's own press/retract ramp gives a per-cell
% Force<->V fit; Capacitance_measurement's LCR sweep gives a per-cell
% Force<->Cp_pF fit; chaining the two through Force eliminates Force and
% leaves this direct V<->Cp_pF equation, per cell, per surface.)
%
% Because Cp is a DIRECT LINEAR FUNCTION of V here (not a separately
% normalized quantity the way V itself is a gamma-transform of raw
% counts), there is no capacitance analogue of "raw counts" as a distinct
% step -- so this produces 4 plots per point, not 5:
%   1. Baseline capacitance   -- Cp at V=0 (that cell's own B coefficient)
%   2. Pressed capacitance    -- Cp at the cross-talk/press V observed
%                                 when the CHOSEN point is pressed
%   3. Delta capacitance      -- Pressed minus Baseline (pF)
%   4. Delta / Baseline       -- Delta as a fraction of Baseline Cp
%
% CAVEAT: A/B were fit from V values recorded while THAT SPECIFIC point
% was itself being pressed (own-press dynamics). Applying that same
% per-cell equation to CROSS-TALK-induced V (small residual signal on a
% cell when a NEIGHBORING point is pressed instead) assumes the cell's
% own V<->Cp transfer function doesn't depend on why V changed -- a
% reasonable but untested extrapolation of the model to a much smaller
% signal regime than it was fit on.
%
% Where the results go: mucaboard_data_raw/matlab/results/P<id>/ -- same
% per-point subfolders simple_19points_analysis.m already writes into,
% with a "capacitance_" prefix on these 4 new files (.fig + .svg).
% =========================================================================

clear; close all; clc;

addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'mucaboard_data', 'matlab'));

%% ---- a tiny helper to turn one 19x3 matrix into a hexagon picture -----
function fig = plot_matrix_as_hexmap(matrix_19x3, surface_names, colorbar_label, title_str, highlight_point, color_limits, show_labels, ids, xy)
    HEX_RADIUS = 8.0 / sqrt(3);
    if isempty(color_limits)
        vmin = min(matrix_19x3(:));
        vmax = max(matrix_19x3(:));
    else
        vmin = color_limits(1);
        vmax = color_limits(2);
    end
    cmap = get_cmap();
    n_panels = numel(surface_names);
    panel_left_all = [0.03, 0.35, 0.67];
    panel_width = 0.27;

    fig = figure('Position', [100 100 500 * n_panels + 100, 550]);
    axes_list = cell(1, n_panels);
    for i = 1:n_panels
        ax = axes('Parent', fig, 'Position', [panel_left_all(min(i,3)), 0.12, panel_width, 0.72]);
        axes_list{i} = ax;
        draw_hex_panel(ax, ids, xy, matrix_19x3(:, i), cmap, vmin, vmax, ...
            HEX_RADIUS, upper(surface_names{i}), highlight_point, show_labels);
    end
    colormap(fig, cmap);
    for i = 1:n_panels
        set(axes_list{i}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{end}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, colorbar_label);
    for i = 1:n_panels
        set(axes_list{i}, 'Position', [panel_left_all(min(i,3)), 0.12, panel_width, 0.72]);
    end
    fig_suptitle(fig, title_str);
end

%% ---- a tiny helper to save a figure as .fig + .svg (no PNG) -----------
function save_fig_svg(fig, path_no_ext)
    fig_path = [path_no_ext '.fig'];
    svg_path = [path_no_ext '.svg'];
    savefig(fig, fig_path);
    print(fig, svg_path, '-dsvg');
    fprintf('  saved: %s\n', fig_path);
    fprintf('  saved: %s\n', svg_path);
end

%% ---- same NaN-aware median/mean-across-columns helpers ---------------
function out_row = median_ignore_nan_cols(M)
    n_cols = size(M, 2);
    out_row = nan(1, n_cols);
    for c = 1:n_cols
        col = M(:, c);
        col = col(~isnan(col));
        if ~isempty(col)
            out_row(c) = median(col);
        end
    end
end

function out_row = nanmean_cols(M)
    n_cols = size(M, 2);
    out_row = nan(1, n_cols);
    for c = 1:n_cols
        col = M(:, c);
        col = col(~isnan(col));
        if ~isempty(col)
            out_row(c) = mean(col);
        end
    end
end

% ---- Board-label / own-cell corrections -- identical to
% simple_19points_analysis.m (see that script for the full derivation).
TRUE_TO_CODE = [8,4,1,13,9,5,2,17,14,10,6,3,18,15,11,7,19,16,12];
[code_point_ids, code_point_xy, ~] = muca_layout();
point_ids    = (1:19)';
point_xy     = code_point_xy(TRUE_TO_CODE, :);
own_cell_col = TRUE_TO_CODE(:);
HEX_RADIUS = 8.0 / sqrt(3);

%% ---- STEP 1: settings -------------------------------------------------
data_folder = fullfile(fileparts(mfilename('fullpath')), '..', 'logs');

surface_names = {'flat', 'solid', 'hollow'};
file_names = { ...
    'flat_sensor_19_points_10_iterations_session_20260826_155255.csv', ...
    'solid_sensor_19_points_10_iterations_session_20260826_165651.csv', ...
    'hollow_sensor_iterations_all_session_20260826_190632.csv' ...
};

% All 19 points, matching the per-point folders already populated by
% simple_19points_analysis.m -- these new capacitance_* files are added
% alongside the existing 5 files in each results/P<id>/ folder.
POINT_IDS_TO_ANALYZE = 1:19;

N_ITERATIONS  = 10;
SENSITIVITY   = 36.0;
GAMMA         = 0.5;
RAW_VALID_MAX = 1000;
SHOW_LABELS   = false;

n_points   = 19;
n_surfaces = 3;

%% ---- STEP 1B: load the per-cell, per-surface Cp = A*V + B table -------
coeff_csv = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
    'Capacitance_calibration_Muca_LCR_Futek', 'results', 'muca_v_to_cp_table.csv');
if ~exist(coeff_csv, 'file')
    error('capacitance_19points_analysis:missing', ...
        ['muca_v_to_cp_table.csv not found -- run, in order:\n' ...
         '  Capacitance_calibration_Muca_LCR_Futek/per_point_surface_relationships.m\n' ...
         '  Capacitance_calibration_Muca_LCR_Futek/chained_muca_v_to_cp_via_force.m\n' ...
         '  Capacitance_calibration_Muca_LCR_Futek/muca_v_to_cp_table.m\n' ...
         'first. Expected at: %s'], coeff_csv);
end
fid = fopen(coeff_csv, 'r');
header_line = fgetl(fid);
header = strsplit(header_line, ',', 'CollapseDelimiters', false);
fmt = repmat('%s', 1, numel(header));
Ccoef = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
fclose(fid);
colc = @(name) Ccoef{find(strcmp(header, name), 1)};
coef_surface = colc('surface');
coef_point   = str2double(colc('true_point'));
coef_A       = str2double(colc('slope_A'));
coef_B       = str2double(colc('intercept_B'));
coef_r2      = str2double(colc('r2_weakest_link'));

A_table  = nan(19, 3);   % A_table(cell, surface)
B_table  = nan(19, 3);
R2_table = nan(19, 3);
for i = 1:numel(coef_surface)
    s = find(strcmp(surface_names, coef_surface{i}), 1);
    p = coef_point(i);
    if isempty(s) || isnan(p) || p < 1 || p > 19
        continue;
    end
    A_table(p, s)  = coef_A(i);
    B_table(p, s)  = coef_B(i);
    R2_table(p, s) = coef_r2(i);
end
fprintf('Loaded muca_V -> Cp_pF coefficients: %d/57 (point,surface) equations, mean R2=%.3f\n', ...
    sum(~isnan(A_table(:))), mean(R2_table(~isnan(R2_table))));

%% ---- STEP 2: read all 3 surface files ONCE (identical to
%       simple_19points_analysis.m) ---------------------------------------
surface_data = cell(1, n_surfaces);
for s = 1:n_surfaces
    csv_path = fullfile(data_folder, file_names{s});
    fprintf('Reading %s ...\n', file_names{s});

    fid = fopen(csv_path, 'r');
    header_line  = fgetl(fid);
    column_names = strsplit(header_line, ',', 'CollapseDelimiters', false);

    text_rows = {};
    while true
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end
        text_rows{end + 1} = strsplit(line, ',', 'CollapseDelimiters', false); %#ok<AGROW>
    end
    fclose(fid);

    n_rows    = numel(text_rows);
    data_text = vertcat(text_rows{:});

    column_index = @(name) find(strcmp(column_names, name));
    get_numbers  = @(name) str2double(data_text(:, column_index(name)));

    ur5_point = get_numbers('ur5_point');
    round_idx = get_numbers('round_idx');
    phase     = data_text(:, column_index('phase'));

    raw_cells = nan(n_rows, 19);
    for k = 1:19
        raw_cells(:, k) = get_numbers(sprintf('cell_%d', k));
    end
    calib_raw = nan(1, 19);
    for k = 1:19
        one_column = get_numbers(sprintf('calib_%d', k));
        calib_raw(k) = one_column(1);
    end

    n_bad = sum(raw_cells(:) > RAW_VALID_MAX);
    raw_cells(raw_cells > RAW_VALID_MAX) = NaN;
    calib_raw(calib_raw > RAW_VALID_MAX) = NaN;
    if n_bad > 0
        fprintf(['  WARNING: %d/%d raw readings exceeded %g raw counts ' ...
            '(sensor glitch) -- set to NaN and excluded from all stats\n'], ...
            n_bad, numel(raw_cells), RAW_VALID_MAX);
    end

    raw_by_point   = raw_cells(:, own_cell_col);
    calib_by_point = calib_raw(own_cell_col);

    surface_data{s} = struct( ...
        'raw_by_point',   raw_by_point, ...
        'calib_by_point', calib_by_point, ...
        'ur5_point',      ur5_point, ...
        'round_idx',      round_idx, ...
        'phase',          {phase});

    fprintf('  %d rows loaded\n', n_rows);
end

%% ---- STEP 3: for every chosen point, build Normalized_matrix (V), ------
%       then convert to Cp_pF using the per-cell/per-surface table -------
results_folder = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

for pi = 1:numel(POINT_IDS_TO_ANALYZE)
    PRESSED_POINT      = POINT_IDS_TO_ANALYZE(pi);
    CODE_PRESSED_POINT = TRUE_TO_CODE(PRESSED_POINT);
    fprintf('\n%s\n', repmat('=', 1, 60));
    fprintf('  POINT %d\n', PRESSED_POINT);
    fprintf('%s\n', repmat('=', 1, 60));

    Normalized_matrix = nan(n_points, n_surfaces);

    for s = 1:n_surfaces
        d = surface_data{s};

        is_pressed_row = (d.ur5_point == CODE_PRESSED_POINT) ...
                        & (d.round_idx >= 0) & (d.round_idx <= N_ITERATIONS - 1) ...
                        & strcmp(d.phase, 'hold');

        ratio_all = (d.raw_by_point - d.calib_by_point) / SENSITIVITY;
        ratio_all = min(max(ratio_all, 0), 1);
        normalized_all = ratio_all .^ GAMMA;

        normalized_per_iteration = nan(N_ITERATIONS, 19);
        for iteration = 0:(N_ITERATIONS - 1)
            rows = is_pressed_row & (d.round_idx == iteration);
            if ~any(rows)
                continue;
            end
            normalized_per_iteration(iteration + 1, :) = nanmean_cols(normalized_all(rows, :));
        end
        Normalized_matrix(:, s) = median_ignore_nan_cols(normalized_per_iteration)';

        found = ~isnan(normalized_per_iteration(:, 1));
        fprintf('%-8s: %d/%d iterations found for point %d\n', ...
            surface_names{s}, sum(found), N_ITERATIONS, PRESSED_POINT);
    end

    % ---- Convert to capacitance, per cell (row), per surface (col) -----
    % Baseline_pF(k,s) = B(k,s) -- Cp at V=0, a fixed property of cell k
    % on surface s, the SAME for every PRESSED_POINT (identical reasoning
    % to why Baseline_matrix in simple_19points_analysis.m never depended
    % on which point was pressed either -- it's calib_by_point there too).
    Baseline_pF = B_table;
    Press_pF    = A_table .* Normalized_matrix + B_table;
    Delta_pF    = Press_pF - Baseline_pF;
    DeltaOverBaseline_pF = Delta_pF ./ Baseline_pF;

    % ---- Independent scales, NOT shared -----------------------------------
    % Baseline_pF (the B intercepts) never changes with PRESSED_POINT --
    % it's a fixed property of each cell/surface. Press_pF DOES change per
    % point (through Normalized_matrix), but only by ~0.1-0.2 pF, while
    % Baseline itself spans ~2 pF cell-to-cell (real electrode-to-electrode
    % hardware variation). Sharing one scale between the two, as this
    % script originally did, caused two problems: (1) Baseline's rendered
    % colors shifted by a rounding sliver from one point-folder to the
    % next, even though the underlying values never changed -- confusing,
    % since it looked like a bug rather than the true no-change reality;
    % (2) Press's real per-point touch signal got crushed into an
    % imperceptible fraction of a color range dominated by Baseline's much
    % larger spread. Giving each its own scale fixes both: Baseline now
    % renders IDENTICALLY across every point folder (as it should, since
    % it doesn't depend on which point was pressed), and Press uses its
    % own full range.
    baseline_scale = [min(Baseline_pF(:)), max(Baseline_pF(:))];

    press_values = Press_pF(~isnan(Press_pF));
    press_scale  = [min(press_values), max(press_values)];

    delta_values = Delta_pF(~isnan(Delta_pF));
    delta_scale  = [min(delta_values), max(delta_values)];

    dob_values = DeltaOverBaseline_pF(~isnan(DeltaOverBaseline_pF));
    dob_scale  = [min(dob_values), max(dob_values)];

    fprintf('  Baseline Cp scale (fixed, same for every point): [%.4f, %.4f] pF\n', baseline_scale(1), baseline_scale(2));
    fprintf('  Press Cp scale (this point only): [%.4f, %.4f] pF\n', press_scale(1), press_scale(2));
    fprintf('  Delta Cp scale: [%.4f, %.4f] pF\n', delta_scale(1), delta_scale(2));
    fprintf('  Delta/Baseline Cp scale: [%.4f, %.4f]\n', dob_scale(1), dob_scale(2));

    point_folder = fullfile(results_folder, sprintf('P%02d', PRESSED_POINT));
    if ~exist(point_folder, 'dir')
        mkdir(point_folder);
    end

    fig1 = plot_matrix_as_hexmap(Baseline_pF, surface_names, ...
        'Baseline capacitance, Cp at V=0 (pF)', ...
        sprintf('P%02d -- baseline capacitance', PRESSED_POINT), PRESSED_POINT, baseline_scale, SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig1, fullfile(point_folder, 'capacitance_baseline_hexmap'));

    fig2 = plot_matrix_as_hexmap(Press_pF, surface_names, ...
        'Pressed capacitance (pF)', ...
        sprintf('P%02d -- pressed capacitance', PRESSED_POINT), PRESSED_POINT, press_scale, SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig2, fullfile(point_folder, 'capacitance_press_hexmap'));

    fig3 = plot_matrix_as_hexmap(Delta_pF, surface_names, ...
        'Delta capacitance = Pressed - Baseline (pF)', ...
        sprintf('P%02d -- delta capacitance', PRESSED_POINT), PRESSED_POINT, [], SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig3, fullfile(point_folder, 'capacitance_delta_hexmap'));

    fig4 = plot_matrix_as_hexmap(DeltaOverBaseline_pF, surface_names, ...
        'Delta Cp / Baseline Cp (fraction)', ...
        sprintf('P%02d -- delta capacitance / baseline capacitance', PRESSED_POINT), PRESSED_POINT, [], SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig4, fullfile(point_folder, 'capacitance_delta_over_baseline_hexmap'));

    close([fig1 fig2 fig3 fig4]);
    fprintf('  Figures saved in: %s\n', point_folder);
end

fprintf('\nAll done. Results in: %s\n', results_folder);
