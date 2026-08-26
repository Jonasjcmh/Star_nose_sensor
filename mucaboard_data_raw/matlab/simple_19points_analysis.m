% =========================================================================
% simple_19points_analysis.m
%
% Same style and same 5 results as mucaboard_data/matlab/
% simple_central_point_analysis.m, but for the NEW "mucaboard_data_raw"
% dataset: 3 files (flat/solid/hollow), each sweeping ALL 19 points (not
% just the board's center), 5 repeats per point.
%
% What's in each CSV row that matters to us (identical layout to the
% central-point dataset -- see mucaboard_data/README_signal_normalization.md):
%   ur5_point   -- which point the robot is currently pressing
%   phase       -- we only care about 'hold' (PRESSED: robot holding the
%                  probe all the way down)
%   round_idx   -- which repeat this is (0 to 4 = the 5 iterations in
%                  THESE files -- NOT 10 like the central-point dataset;
%                  -1 = idle rows before the real presses start)
%   cell_1..19  -- the RAW reading of each of the 19 sensing pads (NOT
%                  yet normalized -- just small integer counts)
%   calib_1..19 -- each pad's resting baseline count (the SAME for every
%                  row in a file -- measured once, at the very start of
%                  the session)
%
% What you get, for EACH point you list in POINT_IDS_TO_ANALYZE below --
% 5 results, 5 plots, same definitions as simple_central_point_analysis.m:
%   1. Baseline   -- that pad's calibration baseline (straight from calib_i)
%   2. Raw Press  -- that pad's RAW reading while the CHOSEN point is PRESSED
%   3. Delta Raw  -- Raw Press minus Baseline (raw-count units)
%   4. Normalized -- the pressed reading converted to the sensor's own
%                    normalized [0,1] scale (same GAMMA/SENSITIVITY formula
%                    as the real firmware), applied SAMPLE BY SAMPLE before
%                    any averaging.
%   5. Delta / Baseline -- Delta Raw as a FRACTION of that pad's own
%                    baseline (dimensionless).
%
% Each of those 5 results is one 19x3 MATRIX per chosen point: 19 rows
% (physical point) x 3 columns (surface). Every one of the 19 columns in
% the raw data is read out for EVERY chosen point -- that's what shows the
% cross-talk spreading out from whichever point was actually pressed (the
% one outlined in red on each plot).
%
% Where the results go: mucaboard_data_raw/matlab/results/P<id>/ -- one
% subfolder PER ANALYZED POINT, containing all 5 plots as BOTH .fig
% (MATLAB/Octave's own re-editable format) and .svg (vector, for
% documents) -- no PNG this time, per what was asked.
%
% Point ids everywhere in this script (POINT_IDS_TO_ANALYZE, plot
% labels/highlight) are the TRUE physical board ids, matching the real
% board's own numbering -- NOT the robot control script's internal
% numbering (what actually ends up in the CSV's ur5_point column). Those
% two numberings turned out not to match; see the TRUE_TO_CODE block
% below for the derivation and translation.
% =========================================================================

clear; close all; clc;

% ---- Reuse the shared hex-plotting/layout toolkit already built for
% mucaboard_data -- no need to reinvent it here.
addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'mucaboard_data', 'matlab'));

%% ---- a tiny helper to turn one 19x3 matrix into a hexagon picture -----
% (defined up top, before first use, on purpose -- Octave requires that)
%
% This does the same 3-panel layout as mucaboard_data/matlab/
% muca_plot_hex_comparison.m, but can't just call that function: it looks
% up point_ids/point_xy from muca_layout() ITSELF, which -- as explained
% by the TRUE_TO_CODE block below -- uses the robot's internal point
% numbering, not the board's true silkscreen numbering. So this takes
% ids/xy as explicit arguments (this script's own corrected versions)
% instead, reusing the same low-level draw_hex_panel/get_cmap/
% fig_suptitle building blocks.
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

%% ---- a tiny helper: median of each COLUMN, ignoring NaN entries -------
% Octave/MATLAB's median() propagates NaN (one NaN in a column -> NaN
% out), so a plain median() over the 5 iteration-rows would wipe out a
% whole point just because one iteration was bad. This instead drops the
% NaNs within each column independently and takes the median of what's
% left (NaN only if every entry in that column is NaN).
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

%% ---- a tiny helper: mean of each COLUMN, ignoring NaN entries ---------
% plain mean() propagates NaN too -- same reasoning as
% median_ignore_nan_cols above, just for averaging the rows of ONE
% iteration's samples instead of averaging across iterations.
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

% ---- Board-label correction (THIS SCRIPT ONLY) -------------------------
% muca_layout.m's point_xy/own_cell_col are indexed by the ROBOT'S
% INTERNAL point numbering (from dataset_collector/collect.py's POINTS
% dict -- this is also exactly what's stored in every CSV's ur5_point
% column). That internal numbering does NOT match the physical board's
% own silkscreen numbering (confirmed against a photo of the real board).
% The mm positions themselves are accurate (that's really where the
% robot pressed) -- only the ID attached to each position is off, by a
% fixed, board-wide relabeling. TRUE_TO_CODE(true_board_id) below is that
% relabeling, worked out slot-by-slot from the photo; everything in THIS
% script from here on uses the true board id, translating to the robot's
% internal id only at the one place that has to match the CSV
% (ur5_point). This is a local fix -- muca_layout.m and every other
% script that calls it are untouched, on purpose.
TRUE_TO_CODE = [8,4,1,13,9,5,2,17,14,10,6,3,18,15,11,7,19,16,12];

% ---- own_cell_col correction (THIS SCRIPT ONLY, THIS DATASET ONLY) -----
% muca_layout.m's own_cell_col (sourced from Integration_2/sensor.py's
% _UR5_TO_IDX) does NOT describe how THIS dataset's cell_1..19 columns
% are laid out -- checked directly against the data: for every one of the
% 19 points, the "hold phase minus locate/post" delta (i.e. the
% press-locked response, isolated from any chronic per-cell offset) is
% BIGGEST on cell_<robot's own point number> itself, not on
% cell_<own_cell_col(point)>. Using muca_layout's own_cell_col here gave
% NEGATIVE "own" deltas for most points while some OTHER point's press
% lit that same cell up -- a strong sign that whatever board/firmware
% revision logged this raw dataset writes cell_k directly as point k's
% own reading (identity), unlike the older mucaboard_data recordings.
% Confirmed on both the flat and hollow files (mean own/cross-talk ratio
% 4.7x and 3.2x with identity vs <=1x with muca_layout's own_cell_col).
[code_point_ids, code_point_xy, ~] = muca_layout();
point_ids    = (1:19)';                        % now TRUE board ids
point_xy     = code_point_xy(TRUE_TO_CODE, :);  % same mm positions, board-id order
own_cell_col = TRUE_TO_CODE(:);                 % cell_<robot id> IS that point's own reading
HEX_RADIUS = 8.0 / sqrt(3);

%% ---- STEP 1: settings -------------------------------------------------
data_folder = fullfile(fileparts(mfilename('fullpath')), '..', 'logs');

surface_names = {'flat', 'solid', 'hollow'};
file_names = { ...
    'flat_sensor_19_points_10_iterations_session_20260826_155255.csv', ...
    'solid_sensor_19_points_10_iterations_session_20260826_165651.csv', ...
    'hollow_sensor_iterations_all_session_20260826_190632.csv' ...
};
% NOTE: the previous hollow file (..._134101.csv) had cells 4-7 pegged
% near 65535 (16-bit overflow) from the very first row -- see
% qa_check_dataset.m. This is the re-collected replacement; it passes
% all 5 QA checks clean (verified 2026-08-26).

% ---- CHOOSE WHICH POINTS TO ANALYZE HERE -------------------------------
% One full set of 5 plots gets generated per point listed here, saved
% into its own results/P<id>/ subfolder. Any of 1-19 -- these are the
% TRUE BOARD ids (matching the physical board's own numbering), not the
% robot's internal numbering; see the TRUE_TO_CODE block below for why
% that distinction exists and matters here.
POINT_IDS_TO_ANALYZE = 1:19;

N_ITERATIONS   = 10;   % round_idx goes 0-9 in these files.
PRESS_DEPTH_MM = 10;    % kept for reference; only one depth exists in
                         % these files, so it isn't filtered on below

% The sensor's own normalization formula (Integration_2/sensor.py):
%   V = clip( (raw - baseline) / SENSITIVITY , 0, 1 ) ^ GAMMA
% See mucaboard_data/README_signal_normalization.md for the derivation.
SENSITIVITY = 36.0;
GAMMA       = 0.5;

% ---- Sanity filter for a real hardware fault found in these files -----
% The "solid" surface log has a genuine sensor glitch: partway through
% that session, raw cells 1 and 4 start reporting ~65500-ish counts (a
% 16-bit wraparound) and stay broken for almost the rest of the file.
% Every real raw reading in all 3 files stays under ~50, so anything
% above this ceiling is definitely bad hardware data, not a real touch --
% it gets turned into NaN right after reading, and every average/median
% below ignores NaNs so one glitchy cell can't blow out the whole plot.
RAW_VALID_MAX = 1000;

% true  -> every hexagon shows "P##" and its value.
% false -> hexagons show color only, no text.
SHOW_LABELS = true;

n_points   = 19;
n_surfaces = 3;

%% ---- STEP 2: read all 3 surface files ONCE ----------------------------
% Each file is ~27,500 rows / ~9 MB -- re-reading it for every point in
% POINT_IDS_TO_ANALYZE would be needlessly slow (each read takes several
% seconds). Read every surface exactly once here; every chosen point
% below re-uses this same in-memory data.
surface_data = cell(1, n_surfaces);
for s = 1:n_surfaces

    csv_path = fullfile(data_folder, file_names{s});
    fprintf('Reading %s ...\n', file_names{s});

    % --- 2a. Read the whole file as TEXT, split on commas -------------
    fid = fopen(csv_path, 'r');
    header_line  = fgetl(fid);
    column_names = strsplit(header_line, ',', 'CollapseDelimiters', false);

    text_rows = {};   % will grow by one row every time through the loop
    while true
        line = fgetl(fid);
        if ~ischar(line)     % fgetl returns -1 (not text) at end of file
            break;
        end
        text_rows{end + 1} = strsplit(line, ',', 'CollapseDelimiters', false); %#ok<AGROW>
    end
    fclose(fid);

    n_rows    = numel(text_rows);
    data_text = vertcat(text_rows{:});

    % --- 2b. Two tiny helpers for pulling one named column out --------
    column_index = @(name) find(strcmp(column_names, name));
    get_numbers  = @(name) str2double(data_text(:, column_index(name)));

    % --- 2c. Read the columns we need ----------------------------------
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

    % --- 2c'. Drop hardware-glitch readings (see RAW_VALID_MAX above) --
    n_bad = sum(raw_cells(:) > RAW_VALID_MAX);
    raw_cells(raw_cells > RAW_VALID_MAX) = NaN;
    calib_raw(calib_raw > RAW_VALID_MAX) = NaN;
    if n_bad > 0
        fprintf(['  WARNING: %d/%d raw readings exceeded %g raw counts ' ...
            '(sensor glitch) -- set to NaN and excluded from all stats\n'], ...
            n_bad, numel(raw_cells), RAW_VALID_MAX);
    end

    % --- 2d. Put the 19 columns into PHYSICAL POINT order --------------
    raw_by_point   = raw_cells(:, own_cell_col);   % n_rows x 19, column p = point p
    calib_by_point = calib_raw(own_cell_col);      % 1 x 19,      column p = point p

    % --- 2e. Stash everything this surface needs for STEP 3 -------------
    surface_data{s} = struct( ...
        'raw_by_point',   raw_by_point, ...
        'calib_by_point', calib_by_point, ...
        'ur5_point',      ur5_point, ...
        'round_idx',      round_idx, ...
        'phase',          {phase});

    fprintf('  %d rows loaded\n', n_rows);
end

%% ---- STEP 3: for every chosen point, build the 5 matrices + 5 plots --
results_folder = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

for pi = 1:numel(POINT_IDS_TO_ANALYZE)
    PRESSED_POINT      = POINT_IDS_TO_ANALYZE(pi);   % TRUE board id (labels/plots)
    CODE_PRESSED_POINT = TRUE_TO_CODE(PRESSED_POINT); % robot's id (ur5_point filter)
    fprintf('\n%s\n', repmat('=', 1, 60));
    fprintf('  POINT %d\n', PRESSED_POINT);
    fprintf('%s\n', repmat('=', 1, 60));

    Baseline_matrix          = nan(n_points, n_surfaces);
    RawPress_matrix          = nan(n_points, n_surfaces);
    DeltaRaw_matrix          = nan(n_points, n_surfaces);
    Normalized_matrix        = nan(n_points, n_surfaces);
    DeltaOverBaseline_matrix = nan(n_points, n_surfaces);

    for s = 1:n_surfaces
        d = surface_data{s};
        Baseline_matrix(:, s) = d.calib_by_point';

        % --- Which rows are "pressed" for THIS point? -------------------
        % ur5_point in the CSV is the robot's internal id, so filter on
        % CODE_PRESSED_POINT (see TRUE_TO_CODE above), not PRESSED_POINT.
        is_pressed_row = (d.ur5_point == CODE_PRESSED_POINT) ...
                        & (d.round_idx >= 0) & (d.round_idx <= N_ITERATIONS - 1) ...
                        & strcmp(d.phase, 'hold');

        % --- Two versions of every pressed sample, built BEFORE any
        %      averaging: raw delta from baseline, and normalized [0,1] --
        delta_from_calib_all = d.raw_by_point - d.calib_by_point;
        ratio_all = (d.raw_by_point - d.calib_by_point) / SENSITIVITY;
        ratio_all = min(max(ratio_all, 0), 1);
        normalized_all = ratio_all .^ GAMMA;

        % --- Average each iteration separately, then take the median ---
        % (nanmean_cols ignores NaN entries column-by-column, so a
        % glitched cell in a few rows of an otherwise-good iteration
        % just gets left out of that average instead of poisoning it.)
        pressed_per_iteration    = nan(N_ITERATIONS, 19);
        delta_per_iteration      = nan(N_ITERATIONS, 19);
        normalized_per_iteration = nan(N_ITERATIONS, 19);
        for iteration = 0:(N_ITERATIONS - 1)
            rows = is_pressed_row & (d.round_idx == iteration);
            if ~any(rows)
                continue;   % this point/round had no hold rows -- leave NaN
            end
            pressed_per_iteration(iteration + 1, :)    = nanmean_cols(d.raw_by_point(rows, :));
            delta_per_iteration(iteration + 1, :)      = nanmean_cols(delta_from_calib_all(rows, :));
            normalized_per_iteration(iteration + 1, :) = nanmean_cols(normalized_all(rows, :));
        end

        % Take the median ACROSS the 5 iterations one column (= one
        % physical point) at a time, ignoring any iteration that came out
        % NaN for that particular point -- a whole missing iteration and
        % a single glitched point within an otherwise-fine iteration are
        % both handled the same way.
        RawPress_matrix(:, s)   = median_ignore_nan_cols(pressed_per_iteration)';
        DeltaRaw_matrix(:, s)   = median_ignore_nan_cols(delta_per_iteration)';
        Normalized_matrix(:, s) = median_ignore_nan_cols(normalized_per_iteration)';

        found = ~isnan(pressed_per_iteration(:, 1));
        fprintf('%-8s: %d/%d iterations found for point %d\n', ...
            surface_names{s}, sum(found), N_ITERATIONS, PRESSED_POINT);
    end

    % Delta / Baseline -- Baseline is a fixed constant per point, so
    % dividing by it commutes exactly with averaging (no sample-by-sample
    % subtlety needed, unlike the gamma normalization above).
    DeltaOverBaseline_matrix = DeltaRaw_matrix ./ Baseline_matrix;

    % ---- Two PAIRS of plots share a legend; DeltaRaw stands alone -----
    %   Baseline + RawPress            -> raw-count unit, share one scale
    %   Normalized + Delta/Baseline    -> dimensionless fraction, share one scale
    %   DeltaRaw                       -> own scale
    baseline_press_values = [Baseline_matrix(:); RawPress_matrix(:)];
    baseline_press_scale  = [min(baseline_press_values), max(baseline_press_values)];

    norm_ratio_values = [Normalized_matrix(:); DeltaOverBaseline_matrix(:)];
    norm_ratio_scale  = [min(norm_ratio_values), max(norm_ratio_values)];

    fprintf('  Baseline + RawPress shared scale: [%.2f, %.2f]\n', baseline_press_scale(1), baseline_press_scale(2));
    fprintf('  Normalized + Delta/Baseline shared scale: [%.4f, %.4f]\n', norm_ratio_scale(1), norm_ratio_scale(2));

    % ---- Save into its own results/P<id>/ subfolder -------------------
    point_folder = fullfile(results_folder, sprintf('P%02d', PRESSED_POINT));
    if ~exist(point_folder, 'dir')
        mkdir(point_folder);
    end

    fig1 = plot_matrix_as_hexmap(Baseline_matrix, surface_names, ...
        'Baseline (raw counts)', ...
        sprintf('P%02d -- baseline (calibration)', PRESSED_POINT), PRESSED_POINT, baseline_press_scale, SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig1, fullfile(point_folder, 'baseline_hexmap'));

    fig2 = plot_matrix_as_hexmap(RawPress_matrix, surface_names, ...
        'Raw signal, pressed (raw counts)', ...
        sprintf('P%02d -- raw signal, pressed', PRESSED_POINT), PRESSED_POINT, baseline_press_scale, SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig2, fullfile(point_folder, 'raw_press_hexmap'));

    fig3 = plot_matrix_as_hexmap(DeltaRaw_matrix, surface_names, ...
        'Delta raw = Pressed - Baseline (raw counts)', ...
        sprintf('P%02d -- delta raw (pressed minus baseline)', PRESSED_POINT), PRESSED_POINT, [], SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig3, fullfile(point_folder, 'delta_raw_hexmap'));

    fig4 = plot_matrix_as_hexmap(Normalized_matrix, surface_names, ...
        'Normalized, pressed (sensor''s own [0,1] scale)', ...
        sprintf('P%02d -- normalized (pressed)', PRESSED_POINT), PRESSED_POINT, norm_ratio_scale, SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig4, fullfile(point_folder, 'normalized_hexmap'));

    fig5 = plot_matrix_as_hexmap(DeltaOverBaseline_matrix, surface_names, ...
        'Delta / Baseline (fraction of baseline)', ...
        sprintf('P%02d -- delta raw / baseline', PRESSED_POINT), PRESSED_POINT, norm_ratio_scale, SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig5, fullfile(point_folder, 'delta_over_baseline_hexmap'));

    close([fig1 fig2 fig3 fig4 fig5]);   % keep the figure count sane when analyzing many points
    fprintf('  Figures saved in: %s\n', point_folder);
end

fprintf('\nAll done. Results in: %s\n', results_folder);
