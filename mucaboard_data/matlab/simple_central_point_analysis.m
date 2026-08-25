% =========================================================================
% simple_central_point_analysis.m
%
% A BEGINNER-FRIENDLY walkthrough of the 3 "raw data central point"
% files. Point 10 (the board's geometric center) was pressed 10 times on
% each of 3 surfaces (flat, solid, hollow dome). No fancy MATLAB/Octave
% tricks are used here on purpose -- every step is spelled out so you can
% follow exactly how the data turns into numbers, and the numbers turn
% into a picture.
%
% What's in each CSV row that matters to us:
%   ur5_point   -- which point the robot is currently pressing
%   phase       -- we only care about 'hold' (PRESSED: robot holding the
%                  probe all the way down) -- other phases (locate/press/
%                  retract/post) are ignored, see the note below
%   round_idx   -- which repeat this is (0 to 9 = the 10 iterations;
%                  -1 = idle rows before the real presses start)
%   cell_1..19  -- the RAW reading of each of the 19 sensing pads (NOT
%                  yet normalized -- just small integer counts)
%   calib_1..19 -- each pad's resting baseline count (the SAME for every
%                  row in a file -- it's measured once, at the very start
%                  of the session)
%
% Why only 'hold' (pressed) rows: the "delta" and "normalized" values
% below are both defined relative to the BASELINE (calib_i), not relative
% to the released ('locate') reading. Since V is DEFINED as
%   V = clip((raw - baseline)/SENSITIVITY, 0, 1) ^ GAMMA
% the baseline's own normalized value is always exactly 0 by construction
% -- so there is nothing released-phase data could add here; the pressed
% reading is the only thing that ever needs normalizing.
%
% What this script computes, for every one of the 19 points, on every
% one of the 3 surfaces -- 4 results, 4 plots:
%   1. Baseline   -- that pad's calibration baseline (straight from calib_i)
%   2. Raw Press  -- that pad's RAW reading while point 10 is PRESSED
%   3. Delta Raw  -- Raw Press minus Baseline (still raw-count units, no
%                    normalization at all)
%   4. Normalized -- the PRESSED reading converted to the sensor's own
%                    normalized [0,1] scale, using the SAME formula the
%                    real firmware uses (see README_signal_normalization.md),
%                    applied SAMPLE BY SAMPLE (to every raw row) before
%                    any averaging -- not to an already-averaged value.
%                    This matters because GAMMA is a nonlinear power, so
%                    "normalize each sample, then average" and "average,
%                    then normalize" give different numbers.
%
% Each of those 4 results is stored as one 19x3 MATRIX: 19 rows (one per
% physical point) x 3 columns (one per surface). Row p, column s is
% "point p, on surface s". That's the only data structure this script
% uses -- no structs, no fancy indexing, just plain matrices.
% =========================================================================

clear; close all; clc;

%% ---- STEP 0: board layout (only needed for drawing the hexagons) -----
% point_xy(p,:)     = that point's (x,y) position in millimeters
% own_cell_col(p)   = which CSV column ("cell_?") is really wired to
%                     point p (the board is wired in a rotated order,
%                     so point 5's own reading is NOT necessarily in
%                     the column named "cell_5"!)
[point_ids, point_xy, own_cell_col] = muca_layout();
HEX_RADIUS = 8.0 / sqrt(3);

%% ---- STEP 1: settings -------------------------------------------------
data_folder = fullfile(fileparts(mfilename('fullpath')), '..', 'raw data central point');

surface_names = {'flat', 'solid', 'hollow'};
file_names = { ...
    '10_iterations_5mm_flat_session_20260819_173407.csv', ...
    '10_iterations_5mm_solid_session_20260819_171044.csv', ...
    '10_iterations_5mm_hollow_dome_session_20260819_165646.csv' ...
};

PRESSED_POINT  = 10;   % the only point pressed in these 3 files
N_ITERATIONS   = 10;   % round_idx goes from 0 to 9
PRESS_DEPTH_MM = 10;   % not used directly below (only one depth exists
                        % in these files), kept here just for reference

% The sensor's own normalization formula (Integration_2/sensor.py):
%   V = clip( (raw - baseline) / SENSITIVITY , 0, 1 ) ^ GAMMA
% See README_signal_normalization.md for the full explanation.
SENSITIVITY = 36.0;
GAMMA       = 0.5;

% true  -> every hexagon shows "P##" and its value, like before.
% false -> hexagons show color only, no text -- a cleaner look once you
%          already know the layout, or for a presentation figure.
% Applies to all 5 plots below.
SHOW_LABELS = false;


n_points   = 19;
n_surfaces = 3;

%% ---- STEP 2: the 5 result matrices, all 19 x 3, empty for now --------
Baseline_matrix        = nan(n_points, n_surfaces);   % calib_i, raw counts
RawPress_matrix        = nan(n_points, n_surfaces);   % raw reading while pressed
DeltaRaw_matrix         = nan(n_points, n_surfaces);   % RawPress - Baseline, raw counts
Normalized_matrix      = nan(n_points, n_surfaces);   % pressed reading, normalized [0,1]
DeltaOverBaseline_matrix = nan(n_points, n_surfaces);  % DeltaRaw / Baseline, dimensionless fraction

%% ---- STEP 3: one surface (one CSV file) at a time ---------------------
for s = 1:n_surfaces

    csv_path = fullfile(data_folder, file_names{s});
    fprintf('Reading %s ...\n', file_names{s});

    % --- 3a. Read the whole file as TEXT, split on commas -------------
    % A CSV file is just comma-separated text. The first line is the
    % column names; every line after that is one row of values.
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
    data_text = vertcat(text_rows{:});   % stack the rows into one big
                                          % n_rows x n_columns cell array

    % --- 3b. Two tiny helpers for pulling one named column out --------
    % column_index('phase')   -> which position 'phase' is at
    % get_numbers('depth_mm') -> that column, converted to numbers
    column_index = @(name) find(strcmp(column_names, name));
    get_numbers  = @(name) str2double(data_text(:, column_index(name)));

    % --- 3c. Read the columns we need ----------------------------------
    ur5_point = get_numbers('ur5_point');             % which point is being pressed
    round_idx = get_numbers('round_idx');              % 0-9, or -1 for idle rows
    phase     = data_text(:, column_index('phase'));   % kept as TEXT: 'locate','hold',...

    % cell_1..cell_19: stack the 19 raw-reading columns side by side into
    % one n_rows x 19 matrix (column k = CSV column "cell_k" for now).
    raw_cells = nan(n_rows, 19);
    for k = 1:19
        raw_cells(:, k) = get_numbers(sprintf('cell_%d', k));
    end

    % calib_1..calib_19: same idea, but this never changes during a
    % session, so one row (the first) is all we need.
    calib_raw = nan(1, 19);
    for k = 1:19
        one_column = get_numbers(sprintf('calib_%d', k));
        calib_raw(k) = one_column(1);
    end

    % --- 3d. Put the 19 columns into PHYSICAL POINT order --------------
    % raw_cells(:,k) is currently CSV column "cell_k", which is wired to
    % whichever point own_cell_col(k) says it is -- NOT necessarily
    % point k. own_cell_col(p) instead tells us "point p's own reading
    % lives in raw CSV column own_cell_col(p)", so re-ordering the
    % columns with own_cell_col makes column p finally mean "point p":
    raw_by_point   = raw_cells(:, own_cell_col);   % n_rows x 19, column p = point p
    calib_by_point = calib_raw(own_cell_col);      % 1 x 19,      column p = point p

    Baseline_matrix(:, s) = calib_by_point';

    % --- 3e. Which rows are "pressed"? (that's the only phase we need) -
    is_pressed_row = (ur5_point == PRESSED_POINT) ...
                    & (round_idx >= 0) & (round_idx <= N_ITERATIONS - 1) ...
                    & strcmp(phase, 'hold');

    % --- 3f. Two versions of every pressed sample, built BEFORE any
    %          averaging: the raw delta from baseline, and the
    %          normalized [0,1] reading. -------------------------------
    % Delta is linear (just a subtraction), so doing it sample-by-sample
    % vs. after averaging gives identical numbers either way -- it's
    % done this way purely for consistency with the normalization below.
    delta_from_calib_all = raw_by_point - calib_by_point;   % n_rows x 19

    % Normalization is NOT linear (GAMMA is a power < 1), so this one
    % genuinely must happen sample-by-sample, matching how the real
    % sensor firmware computes V for every raw frame in real time,
    % before this script ever sees the data:
    ratio_all = (raw_by_point - calib_by_point) / SENSITIVITY;
    ratio_all = min(max(ratio_all, 0), 1);   % the "clip(...,0,1)" part
    normalized_all = ratio_all .^ GAMMA;      % the "^ GAMMA" part -- one V per RAW SAMPLE

    % --- 3g. Average each iteration separately, then take the median ---
    % (Averaging every round's rows together in one big pool would let a
    % round that logged more samples count more than a round that logged
    % fewer -- doing each iteration on its own first avoids that. Taking
    % the MEDIAN across the 10 iteration-results, instead of a mean, is
    % robust to any single iteration being an outlier.)
    pressed_per_iteration     = nan(N_ITERATIONS, 19);
    delta_per_iteration       = nan(N_ITERATIONS, 19);
    normalized_per_iteration  = nan(N_ITERATIONS, 19);
    for iteration = 0:(N_ITERATIONS - 1)
        rows = is_pressed_row & (round_idx == iteration);
        pressed_per_iteration(iteration + 1, :)    = mean(raw_by_point(rows, :), 1);
        delta_per_iteration(iteration + 1, :)      = mean(delta_from_calib_all(rows, :), 1);
        normalized_per_iteration(iteration + 1, :) = mean(normalized_all(rows, :), 1);
    end

    RawPress_matrix(:, s)   = median(pressed_per_iteration, 1)';
    DeltaRaw_matrix(:, s)   = median(delta_per_iteration, 1)';
    Normalized_matrix(:, s) = median(normalized_per_iteration, 1)';
end

% DeltaRaw / Baseline -- how big the press-induced change is AS A
% FRACTION of that point's own baseline (e.g. 0.50 = the raw signal rose
% by 50% of its resting value). Baseline is a fixed constant per point
% (not a per-sample value), so dividing by it commutes exactly with
% averaging -- computing this from the two already-finished matrices
% below gives the identical result as doing it sample-by-sample would.
DeltaOverBaseline_matrix = DeltaRaw_matrix ./ Baseline_matrix;

fprintf('\nDone reading. Every result below is a 19 (points) x 3 (surfaces) matrix:\n');
fprintf('  Baseline_matrix, RawPress_matrix, DeltaRaw_matrix, Normalized_matrix,\n');
fprintf('  DeltaOverBaseline_matrix\n\n');

%% ---- a tiny helper to turn one 19x3 matrix into a hexagon picture -----
% muca_plot_hex_comparison.m (already in this folder) draws 3 hexagon
% boards side by side from a struct with .flat/.solid/.hollow fields, so
% this helper just copies our matrix's 3 columns into that shape.
function fig = plot_matrix_as_hexmap(matrix_19x3, surface_names, colorbar_label, title_str, highlight_point, color_limits, show_labels)
    values.(surface_names{1}) = matrix_19x3(:, 1);
    values.(surface_names{2}) = matrix_19x3(:, 2);
    values.(surface_names{3}) = matrix_19x3(:, 3);
    fig = muca_plot_hex_comparison(values, colorbar_label, title_str, highlight_point, color_limits, show_labels);
end

%% ---- STEP 4: draw the 4 hexagon diagrams ------------------------------
results_folder = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

% Two PAIRS of plots now share a legend (color scale) with each other;
% the 5th (DeltaRaw) stands alone on its own scale:
%   fig1 Baseline   \__ same raw-count unit -> share one scale
%   fig2 RawPress   /
%   fig4 Normalized          \__ same dimensionless-fraction "shape" of
%   fig5 Delta/Baseline      /   quantity -> share one scale
%   fig3 DeltaRaw   -- not paired with anything, own scale
baseline_press_values = [Baseline_matrix(:); RawPress_matrix(:)];
baseline_press_scale  = [min(baseline_press_values), max(baseline_press_values)];
fprintf('Baseline + RawPress shared scale: [%.2f, %.2f]\n', baseline_press_scale(1), baseline_press_scale(2));

norm_ratio_values = [Normalized_matrix(:); DeltaOverBaseline_matrix(:)];
norm_ratio_scale  = [min(norm_ratio_values), max(norm_ratio_values)];
fprintf('Normalized + Delta/Baseline shared scale: [%.4f, %.4f]\n', norm_ratio_scale(1), norm_ratio_scale(2));

fig1 = plot_matrix_as_hexmap(Baseline_matrix, surface_names, ...
    'Baseline (raw counts)', ...
    'Central-point dataset -- baseline (calibration)', PRESSED_POINT, baseline_press_scale, SHOW_LABELS);
save_fig(fig1, fullfile(results_folder, 'simple_baseline_hexmap'));

fig2 = plot_matrix_as_hexmap(RawPress_matrix, surface_names, ...
    'Raw signal, pressed (raw counts)', ...
    'Central-point dataset -- raw signal, pressed', PRESSED_POINT, baseline_press_scale, SHOW_LABELS);
save_fig(fig2, fullfile(results_folder, 'simple_raw_press_hexmap'));

fig3 = plot_matrix_as_hexmap(DeltaRaw_matrix, surface_names, ...
    'Delta raw = Pressed - Baseline (raw counts)', ...
    'Central-point dataset -- delta raw (pressed minus baseline)', PRESSED_POINT, [], SHOW_LABELS);
save_fig(fig3, fullfile(results_folder, 'simple_delta_raw_hexmap'));

fig4 = plot_matrix_as_hexmap(Normalized_matrix, surface_names, ...
    'Normalized, pressed (sensor''s own [0,1] scale)', ...
    'Central-point dataset -- normalized (pressed)', PRESSED_POINT, norm_ratio_scale, SHOW_LABELS);
save_fig(fig4, fullfile(results_folder, 'simple_normalized_hexmap'));

fig5 = plot_matrix_as_hexmap(DeltaOverBaseline_matrix, surface_names, ...
    'Delta / Baseline (fraction of baseline)', ...
    'Central-point dataset -- delta raw / baseline', PRESSED_POINT, norm_ratio_scale, SHOW_LABELS);
save_fig(fig5, fullfile(results_folder, 'simple_delta_over_baseline_hexmap'));

fprintf('\nRanges actually used:\n');
fprintf('  Baseline:   [%.2f, %.2f]\n', min(Baseline_matrix(:)), max(Baseline_matrix(:)));
fprintf('  RawPress:   [%.2f, %.2f]\n', min(RawPress_matrix(:)), max(RawPress_matrix(:)));
fprintf('  DeltaRaw:   [%.2f, %.2f]\n', min(DeltaRaw_matrix(:)), max(DeltaRaw_matrix(:)));
fprintf('  Delta/Baseline: [%.4f, %.4f]\n', min(DeltaOverBaseline_matrix(:)), max(DeltaOverBaseline_matrix(:)));
fprintf('  Normalized: [%.4f, %.4f]\n', min(Normalized_matrix(:)), max(Normalized_matrix(:)));

fprintf('\nFigures saved in: %s\n', results_folder);
