% =========================================================================
% capacitance_19points_analysis.m
%
% Cross-talk hex maps in REAL CAPACITANCE (pF), for mucaboard_data_raw.
%
% METHOD (rewritten -- see "WHY THIS REPLACED THE OLD METHOD" below):
% converts the muca board's raw counts to capacitance using the DIRECT
% electrical calibration in capacitance_combination_calibration/ -- known
% capacitors wired straight to muca board inputs, with their true value
% measured on an LCR-6100 at the same time. That gives a single, clean
% instrument gain:
%
%     dCp_pF = GAIN * d(raw counts)          GAIN = -0.001744 pF/count
%
% obtained by fitting Cp_pF vs the muca reading across 6 capacitor
% combinations spanning 0.98-2.53 pF (R^2 = 0.9958).
%
% IMPORTANT -- the combination-calibration readings are SIGNED 16-bit
% values that the logger writes as UNSIGNED, so a real reading of -354
% appears in the CSV as 65182 (= -354 + 65536). Reinterpreting them
% (value - 65536 when value > 32767) makes them vary perfectly
% monotonically with the known capacitance; without that step they look
% like meaningless "saturated" garbage near 65535 and get discarded.
% mucaboard_data_raw's own files are unaffected (all small positives).
%
% WHY THIS REPLACED THE OLD METHOD
% The previous version chained two fits through FORCE:
%     V = a*Force + b   (from mucaboard_data_raw + Futek load cell)
%     Cp = c*Force + d  (from Capacitance_measurement + Futek load cell)
% and composed them into Cp = A*V + B by evaluating at V=0 and V=1. That
% turned out to be invalid:
%   - V=0 mapped to a NEGATIVE Futek force (median -4 N) for 44 of 57
%     cells, and V=1 mapped to 26-67 N, while the LCR sweep only ever
%     measured 2.6-20 N. Both anchor points were pure extrapolation.
%   - The resulting "baseline" B ranged 0.05-2.03 pF, which is why the
%     baseline/press hex maps looked saturated and implausible.
%   - 3 cells came out with a sign-flipped slope (from Force<->V fits
%     with R^2 as low as 0.16), mixing +/- into the delta color scale.
%   - Forcing the Force<->V fit through the origin (physically required:
%     zero force must mean zero reading) made R^2 go NEGATIVE (-0.35,
%     -1.28) on several points -- i.e. V-vs-force is genuinely nonlinear
%     and no linear model extrapolates safely to V=0.
% The direct calibration used here needs none of that: no force, no
% cross-dataset bridging, no extrapolation to unphysical states.
%
% INDEPENDENT CORROBORATION
% Applying GAIN to mucaboard_data_raw's own-cell press deltas gives a
% mean dCp of -0.025 pF (range -0.009 to -0.041). Measuring the same
% thing completely independently -- LCR wired to the sensor, Cp change
% from a 5mm to a 10mm press (Capacitance_measurement) -- gives -0.061 pF
% (range -0.040 to -0.083). Same sign, same order of magnitude, from two
% methods that share no data and no assumptions.
%
% WHAT THIS SCRIPT DOES *NOT* CLAIM
% Only the CHANGE in capacitance (dCp) is derived here, never an absolute
% per-cell Cp. The combination calibration measures a bare capacitor
% across an input; the sensor electrode has its own self-capacitance in a
% different topology, so the calibration's absolute intercept does not
% transfer. dCp is offset-free by construction (the baseline cancels in
% the subtraction), which is exactly the quantity a cross-talk map should
% show.
%
% Output: results/P<id>/capacitance_delta_hexmap.(fig|svg) -- one per
% analyzed point, alongside the existing raw-count plots.
% =========================================================================

clear; close all; clc;

addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'mucaboard_data', 'matlab'));

%% ---- helpers (defined up top -- Octave requires that) ----------------
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

function save_fig_svg(fig, path_no_ext)
    savefig(fig, [path_no_ext '.fig']);
    print(fig, [path_no_ext '.svg'], '-dsvg');
    fprintf('  saved: %s.(fig/svg)\n', path_no_ext);
end

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

function v = to_signed16(v)
% Logger writes signed 16-bit readings as unsigned -- undo that.
    v(v > 32767) = v(v > 32767) - 65536;
end

%% ---- board-label / own-cell corrections (see simple_19points_analysis.m)
TRUE_TO_CODE = [8,4,1,13,9,5,2,17,14,10,6,3,18,15,11,7,19,16,12];
[~, code_point_xy, ~] = muca_layout();
point_ids    = (1:19)';
point_xy     = code_point_xy(TRUE_TO_CODE, :);
own_cell_col = TRUE_TO_CODE(:);

%% ---- STEP 1: derive GAIN from the combination calibration -------------
HERE = fileparts(mfilename('fullpath'));
combo_csv = fullfile(HERE, '..', '..', 'capacitance_combination_calibration', 'logs', ...
    'flat_calibration_muca_lcr_session_20260826_225636.csv');

fprintf('%s\n', repmat('=', 1, 70));
fprintf('  STEP 1 -- direct calibration: known capacitor -> muca raw count\n');
fprintf('%s\n', repmat('=', 1, 70));

fid = fopen(combo_csv, 'r');
hl = fgetl(fid);
hdr = strsplit(hl, ',', 'CollapseDelimiters', false);
Cc = textscan(fid, repmat('%s', 1, numel(hdr)), 'Delimiter', ',', 'Whitespace', '');
fclose(fid);
cget = @(n) Cc{find(strcmp(hdr, n), 1)};
c_phase = cget('phase');
c_label = cget('combination_label');
c_cp    = str2double(cget('Cp_pF'));
c_cells = nan(numel(c_phase), 19);
for k = 1:19
    c_cells(:, k) = str2double(cget(sprintf('cell_%d', k)));
end

% Each combination was measured at SEVERAL point_labels in turn (the
% capacitor is moved by hand from one muca input to the next). Those must
% be grouped SEPARATELY: pooling a combination's rows together would
% average the one wired cell's strong reading against the 4-ish blocks
% where that same cell was back at its ~20-count resting level, diluting
% the calibration signal ~5x (and inflating the fitted gain by the same
% factor). Group by (combination, point_label), find the wired cell in
% each group, and use one calibration sample per group.
c_point = cget('point_label');
cal_raw = []; cal_cp = []; cal_lbl = {};
labels = unique(c_label);
fprintf('  %-10s %-10s %12s %14s\n', 'combo', 'point', 'LCR Cp (pF)', 'muca (signed)');
for i = 1:numel(labels)
    lb = labels{i};
    m_lcr = strcmp(c_phase, 'lcr') & strcmp(c_label, lb);
    if ~any(m_lcr)
        continue;
    end
    cp_true = mean(c_cp(m_lcr), 'omitnan');

    pts_here = unique(c_point(strcmp(c_phase, 'muca') & strcmp(c_label, lb)));
    for j = 1:numel(pts_here)
        pl = pts_here{j};
        m_muca = strcmp(c_phase, 'muca') & strcmp(c_label, lb) & strcmp(c_point, pl);
        if ~any(m_muca)
            continue;
        end
        blk = to_signed16(c_cells(m_muca, :));
        cellmeans = nanmean_cols(blk);
        [raw_active, active_k] = min(cellmeans);   % wired cell reads strongly NEGATIVE
        if raw_active > -20   % no clearly-wired cell in this block -- skip
            continue;
        end
        cal_raw(end+1) = raw_active; %#ok<AGROW>
        cal_cp(end+1)  = cp_true;    %#ok<AGROW>
        cal_lbl{end+1} = sprintf('%s/%s', lb, pl); %#ok<AGROW>
        fprintf('  %-10s %-10s %12.4f %14.1f\n', lb, pl, cp_true, raw_active);
    end
end

pcal = polyfit(cal_raw, cal_cp, 1);
yfit = polyval(pcal, cal_raw);
r2cal = 1 - sum((cal_cp - yfit).^2) / sum((cal_cp - mean(cal_cp)).^2);
GAIN = pcal(1);
fprintf('\n  FIT: Cp_pF = %.6f * signed_raw + %.4f   (R2 = %.4f, n = %d)\n', ...
    pcal(1), pcal(2), r2cal, numel(cal_raw));
fprintf('  GAIN = %.6f pF per raw count -- this is what converts a muca\n', GAIN);
fprintf('  reading CHANGE into a capacitance CHANGE.\n');
fprintf(['  NOTE: only the SLOPE is used below. The intercept (%.4f pF) belongs to\n' ...
    '  the bare-capacitor-on-input topology and does NOT transfer to the sensor\n' ...
    '  electrode, so no absolute Cp is claimed -- only dCp, which is offset-free.\n'], pcal(2));

%% ---- STEP 2: settings -------------------------------------------------
data_folder = fullfile(HERE, '..', 'logs');
surface_names = {'flat', 'solid', 'hollow'};
file_names = { ...
    'flat_sensor_19_points_10_iterations_session_20260826_155255.csv', ...
    'solid_sensor_19_points_10_iterations_session_20260826_165651.csv', ...
    'hollow_sensor_iterations_all_session_20260826_190632.csv' ...
};

POINT_IDS_TO_ANALYZE = 1:19;
N_ITERATIONS  = 10;
RAW_VALID_MAX = 1000;   % after signed reinterpretation, anything still this
                         % large is a genuine hardware fault (see qa_check_dataset.m)
SHOW_LABELS   = false;
n_points = 19; n_surfaces = 3;

%% ---- STEP 3: read all 3 surfaces once ---------------------------------
surface_data = cell(1, n_surfaces);
for s = 1:n_surfaces
    csv_path = fullfile(data_folder, file_names{s});
    fprintf('\nReading %s ...\n', file_names{s});
    fid = fopen(csv_path, 'r');
    hl = fgetl(fid);
    cn = strsplit(hl, ',', 'CollapseDelimiters', false);
    tr = {};
    while true
        L = fgetl(fid);
        if ~ischar(L), break; end
        tr{end+1} = strsplit(L, ',', 'CollapseDelimiters', false); %#ok<AGROW>
    end
    fclose(fid);
    n_rows = numel(tr);
    dt = vertcat(tr{:});
    ci = @(n) find(strcmp(cn, n));
    gn = @(n) str2double(dt(:, ci(n)));

    ur5_point = gn('ur5_point');
    round_idx = gn('round_idx');
    phase     = dt(:, ci('phase'));

    raw_cells = nan(n_rows, 19);
    for k = 1:19
        raw_cells(:, k) = to_signed16(gn(sprintf('cell_%d', k)));
    end
    calib_raw = nan(1, 19);
    for k = 1:19
        oc = to_signed16(gn(sprintf('calib_%d', k)));
        calib_raw(k) = oc(1);
    end
    n_bad = sum(abs(raw_cells(:)) > RAW_VALID_MAX);
    raw_cells(abs(raw_cells) > RAW_VALID_MAX) = NaN;
    calib_raw(abs(calib_raw) > RAW_VALID_MAX) = NaN;
    if n_bad > 0
        fprintf('  WARNING: %d readings still out of range after signed fix -- set to NaN\n', n_bad);
    end

    surface_data{s} = struct( ...
        'raw_by_point',   raw_cells(:, own_cell_col), ...
        'calib_by_point', calib_raw(own_cell_col), ...
        'ur5_point',      ur5_point, ...
        'round_idx',      round_idx, ...
        'phase',          {phase});
    fprintf('  %d rows loaded\n', n_rows);
end

%% ---- STEP 4: per point, dCp = GAIN * d(raw) ---------------------------
results_folder = fullfile(HERE, 'results');
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

for pi = 1:numel(POINT_IDS_TO_ANALYZE)
    PRESSED_POINT      = POINT_IDS_TO_ANALYZE(pi);
    CODE_PRESSED_POINT = TRUE_TO_CODE(PRESSED_POINT);
    fprintf('\n%s\n', repmat('=', 1, 60));
    fprintf('  POINT %d\n', PRESSED_POINT);
    fprintf('%s\n', repmat('=', 1, 60));

    DeltaCp = nan(n_points, n_surfaces);

    for s = 1:n_surfaces
        d = surface_data{s};
        is_pressed = (d.ur5_point == CODE_PRESSED_POINT) ...
                   & (d.round_idx >= 0) & (d.round_idx <= N_ITERATIONS - 1) ...
                   & strcmp(d.phase, 'hold');

        % raw delta from that cell's own resting baseline -- the offset is
        % removed HERE, by subtraction, so what follows is purely the
        % press-induced change (exactly what a cross-talk map should show)
        delta_raw_all = d.raw_by_point - d.calib_by_point;

        per_it = nan(N_ITERATIONS, 19);
        for it = 0:(N_ITERATIONS - 1)
            rows = is_pressed & (d.round_idx == it);
            if ~any(rows), continue; end
            per_it(it + 1, :) = nanmean_cols(delta_raw_all(rows, :));
        end
        delta_raw = median_ignore_nan_cols(per_it);
        DeltaCp(:, s) = (GAIN * delta_raw)';

        fprintf('%-8s: %d/%d iterations; raw delta %.1f..%.1f counts -> dCp %.4f..%.4f pF\n', ...
            surface_names{s}, sum(~isnan(per_it(:, 1))), N_ITERATIONS, ...
            min(delta_raw), max(delta_raw), min(GAIN * delta_raw), max(GAIN * delta_raw));
    end

    vals = DeltaCp(~isnan(DeltaCp));
    scale = [min(vals), max(vals)];
    fprintf('  dCp scale: [%.4f, %.4f] pF\n', scale(1), scale(2));

    point_folder = fullfile(results_folder, sprintf('P%02d', PRESSED_POINT));
    if ~exist(point_folder, 'dir')
        mkdir(point_folder);
    end

    fig = plot_matrix_as_hexmap(DeltaCp, surface_names, ...
        'Delta capacitance (pF), press-induced', ...
        sprintf('P%02d -- delta capacitance (direct calibration, GAIN=%.6f pF/count)', PRESSED_POINT, GAIN), ...
        PRESSED_POINT, scale, SHOW_LABELS, point_ids, point_xy);
    save_fig_svg(fig, fullfile(point_folder, 'capacitance_delta_hexmap'));
    close(fig);
end

fprintf('\nAll done. Results in: %s\n', results_folder);
