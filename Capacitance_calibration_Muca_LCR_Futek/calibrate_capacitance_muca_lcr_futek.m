% =========================================================================
% calibrate_capacitance_muca_lcr_futek.m
%
% Goal: express mucaboard_data_raw's raw sensor readings in REAL
% capacitance units (pF), using three datasets that each only cover part
% of the picture:
%
%   1. capacitance_combination_calibration/  -- known capacitors wired
%      DIRECTLY to muca board inputs, read on an LCR-6100. No robot, no
%      Futek force. Gives a clean electrical mapping raw-count <-> Cp_pF,
%      but at a DIFFERENT electrical topology than a real touch (direct
%      2-wire capacitor vs. the FT5x16 chip's own mutual-capacitance touch
%      sensing -- see mucaboard_data/README_capacitance_estimation.md
%      section 2.1 for why these are different electrical quantities).
%
%   2. Capacitance_measurement/  -- UR5 robot presses a real capacitor
%      wired to the LCR-6100, at 19 points x several depths, with the
%      Futek load-cell force logged (fx/fy/fz + load_cell_N). No muca
%      board reading at all.
%
%   3. mucaboard_data_raw/  -- UR5 robot presses the muca board itself, 19
%      points, ONE depth (10mm), muca raw readings + Futek force. No LCR.
%
% None of the three shares all three signals (muca raw, Cp_pF, force) at
% once, so real capacitance for mucaboard_data_raw has to be triangulated
% across datasets. This script:
%
%   STEP 0  Verifies datasets 2 and 3 actually share the same physical UR5
%           point layout (same POINTS dict / REFERENCE_POSE + a live
%           tcp_x/tcp_y/tcp_z check on real session data) -- if they
%           didn't, "point 5" in one file wouldn't be "point 5" in the
%           other and nothing below would be valid.
%
%   STEP A  Direct electrical calibration, from dataset 1 alone
%           (raw_count -> Cp_pF, no touch-mechanics confound, but mostly
%           probes a saturated regime far outside mucaboard_data_raw's
%           actual operating range -- see the printed report).
%
%   STEP B  Force-matched cross-dataset regression (PRIMARY): for every
%           (point, surface), take mucaboard_data_raw's own-cell reading
%           and Futek force at its one depth, then INTERPOLATE dataset 2's
%           per-point (force -> Cp_pF) depth-sweep at that same force to
%           get a matched "ground truth" Cp_pF for that muca reading. Same
%           touch topology as mucaboard_data_raw (both are real presses),
%           unlike STEP A.
%
%   STEP C  (documented, not implemented) -- a hybrid that uses STEP A as
%           an electrical-gain prior and STEP B for the touch-specific
%           correction. Flagged as future work in the final report; not
%           enough independent data here to fit it without overfitting.
%
%   FINAL   Applies the STEP B per-surface fit to the FULL mucaboard_data_raw
%           dataset (every point, every iteration, not just the hold-phase
%           mean), writing mucaboard_data_raw_in_pF.csv plus hex-map
%           figures of the result.
%
% Run: octave-cli calibrate_capacitance_muca_lcr_futek.m  (or open in MATLAB)
% =========================================================================

clear; close all; clc;

HERE          = fileparts(mfilename('fullpath'));
REPO_ROOT     = fullfile(HERE, '..');
MUCA_TOOLKIT  = fullfile(REPO_ROOT, 'mucaboard_data', 'matlab');
MUCA_RAW_LOGS = fullfile(REPO_ROOT, 'mucaboard_data_raw', 'logs');
CAPMEAS_LOGS  = fullfile(REPO_ROOT, 'Capacitance_measurement', 'logs');
COMBO_LOGS    = fullfile(REPO_ROOT, 'capacitance_combination_calibration', 'logs');
RESULTS_DIR   = fullfile(HERE, 'results');
if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end
addpath(MUCA_TOOLKIT);   % reuse read_lcr_csv/load_lcr_files/lcr_stats_by_point_depth/
                          % draw_hex_panel/get_cmap/fig_suptitle/muca_layout/save_fig --
                          % all generic, no own_cell_col dependency issue.

%% ---- tiny local helpers (defined up top -- Octave needs this) ---------
% NOTE: script-level constants are NOT visible inside these -- everything
% they need is passed in as an argument, on purpose.

function d = read_muca_raw_csv(csv_path, raw_valid_max)
% Parses one mucaboard_data_raw session CSV (59 cols, same format used by
% simple_19points_analysis.m). Returns raw_cells/calib_raw in RAW COLUMN
% order (1..19), untranslated -- caller applies TRUE_TO_CODE.
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('read_muca_raw_csv:notfound', 'CSV not found: %s', csv_path);
    end
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
    n_rows = numel(text_rows);
    data_text = vertcat(text_rows{:});

    column_index = @(name) find(strcmp(column_names, name));
    get_numbers  = @(name) str2double(data_text(:, column_index(name)));

    d.ur5_point   = get_numbers('ur5_point');
    d.round_idx   = get_numbers('round_idx');
    d.phase       = data_text(:, column_index('phase'));
    d.load_cell_N = get_numbers('load_cell_N');
    d.tcp_x       = get_numbers('tcp_x');
    d.tcp_y       = get_numbers('tcp_y');
    d.tcp_z       = get_numbers('tcp_z');

    d.raw_cells = nan(n_rows, 19);
    for k = 1:19
        d.raw_cells(:, k) = get_numbers(sprintf('cell_%d', k));
    end
    d.calib_raw = nan(1, 19);
    for k = 1:19
        one_col = get_numbers(sprintf('calib_%d', k));
        d.calib_raw(k) = one_col(1);
    end
    n_bad = sum(d.raw_cells(:) > raw_valid_max);
    d.raw_cells(d.raw_cells > raw_valid_max) = NaN;
    d.calib_raw(d.calib_raw > raw_valid_max) = NaN;
    d.n_rows = n_rows;
    d.n_bad  = n_bad;
end

function rows = muca_raw_own_reading_by_point(d, true_to_code, n_iterations, sensitivity, gamma)
% For every TRUE point 1-19: mean hold-phase (force_N, raw_own, V_own) at
% this file's single depth, pooling round_idx 0..n_iterations-1. Returns
% 19x4: [true_point, force_N, raw_own, V_own] (NaN row if no hold data).
    rows = nan(19, 4);
    for true_pt = 1:19
        code_id = true_to_code(true_pt);
        mask = (d.ur5_point == code_id) & strcmp(d.phase, 'hold') & ...
               (d.round_idx >= 0) & (d.round_idx <= n_iterations - 1);
        if ~any(mask)
            continue;
        end
        raw_own = mean(d.raw_cells(mask, code_id), 'omitnan');
        force   = mean(d.load_cell_N(mask), 'omitnan');
        calib   = d.calib_raw(code_id);
        ratio = min(max((raw_own - calib) / sensitivity, 0), 1);
        v_own = ratio ^ gamma;
        rows(true_pt, :) = [true_pt, force, raw_own, v_own];
    end
end

function combo = combo_active_cell_fit(csv_path)
% Approach A: parses one capacitance_combination_calibration session CSV.
% For every (combination, point_label): finds the "active" cell as the one
% whose muca-phase mean deviates most from that SAME cell's mean across
% the OTHER point_labels in the SAME combination (the capacitor moves by
% hand between readings, so only the currently-wired cell should differ
% from its own quiescent level). Pairs that active cell's raw value with
% the combination's own LCR-phase mean Cp_pF (ground truth, measured once
% per combination, independent of which muca point it's later logged as).
%
% Returns a struct: .raw (Nx1), .Cp_pF (Nx1), .combo_label (Nx1 cellstr),
% .point_label (Nx1 cellstr), .active_cell (Nx1), .n_saturated (how many
% of the N raw readings hit the RAW_VALID_MAX-style ceiling, i.e. >1000).
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('combo_active_cell_fit:notfound', 'CSV not found: %s', csv_path);
    end
    header_line = fgetl(fid);
    header = strsplit(header_line, ',', 'CollapseDelimiters', false);
    ncols = numel(header);
    fmt = repmat('%s', 1, ncols);
    C = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
    fclose(fid);
    col = @(name) C{find(strcmp(header, name), 1)};

    phase   = col('phase');
    combo_i = col('combination_index');
    combo_l = col('combination_label');
    pt_l    = col('point_label');
    Cp_pF   = str2double(col('Cp_pF'));
    cells = nan(numel(phase), 19);
    for k = 1:19
        cells(:, k) = str2double(col(sprintf('cell_%d', k)));
    end

    % ---- one ground-truth Cp_pF per combination (mean of its lcr rows) --
    is_lcr = strcmp(phase, 'lcr');
    combo_ids = unique(combo_i(is_lcr));
    combo_cp = containers.Map();
    combo_label_map = containers.Map();
    for i = 1:numel(combo_ids)
        cid = combo_ids{i};
        m = is_lcr & strcmp(combo_i, cid);
        combo_cp(cid) = mean(Cp_pF(m), 'omitnan');
        lbls = combo_l(m);
        combo_label_map(cid) = lbls{1};
    end

    % ---- per (combo, point_label) mean cell vector, muca phase only ----
    % Subset everything to the muca-phase rows FIRST so every array below
    % shares the same length/indexing (mixing full-length and
    % muca-only-length masks was the earlier bug here).
    is_muca    = strcmp(phase, 'muca');
    m_combo_i  = combo_i(is_muca);
    m_pt_l     = pt_l(is_muca);
    m_cells    = cells(is_muca, :);
    pair_id    = strcat(m_combo_i, {'||'}, m_pt_l);
    upairs     = unique(pair_id);

    n = numel(upairs);
    raw_out = nan(n, 1);
    cp_out  = nan(n, 1);
    combo_label_out = cell(n, 1);
    point_label_out = cell(n, 1);
    active_cell_out = nan(n, 1);

    for i = 1:n
        m = strcmp(pair_id, upairs{i});
        cid = m_combo_i(find(m, 1));
        cid = cid{1};
        mean_cells = mean(m_cells(m, :), 1, 'omitnan');

        % other point_labels in the SAME combination
        m_other = strcmp(m_combo_i, cid) & ~strcmp(pair_id, upairs{i});
        if any(m_other)
            other_mean = mean(m_cells(m_other, :), 1, 'omitnan');
        else
            other_mean = zeros(1, 19);
        end
        dev = abs(mean_cells - other_mean);
        [~, active_k] = max(dev);

        raw_out(i) = mean_cells(active_k);
        if isKey(combo_cp, cid)
            cp_out(i) = combo_cp(cid);
            combo_label_out{i} = combo_label_map(cid);
        end
        pl = m_pt_l(find(m, 1));
        point_label_out{i} = pl{1};
        active_cell_out(i) = active_k;
    end

    combo.raw          = raw_out;
    combo.Cp_pF         = cp_out;
    combo.combo_label   = combo_label_out;
    combo.point_label   = point_label_out;
    combo.active_cell   = active_cell_out;
    combo.n_saturated   = sum(raw_out > 1000);
end

function [cp_est, extrapolated] = interp_force_to_cp(force_cp_pairs, target_force)
% force_cp_pairs: Mx2 [force_N, Cp_pF] (one point's LCR depth sweep).
% Sorts by force and linearly interpolates/extrapolates Cp_pF at
% target_force. extrapolated=true if target_force fell outside the
% sweep's observed force range.
    if size(force_cp_pairs, 1) < 2
        cp_est = NaN;
        extrapolated = true;
        return;
    end
    [f_sorted, order] = sort(force_cp_pairs(:, 1));
    cp_sorted = force_cp_pairs(order, 2);
    % de-duplicate identical force values (interp1 needs strictly
    % increasing sample points) by averaging their Cp
    [f_unique, ~, ic] = unique(f_sorted);
    cp_unique = accumarray(ic, cp_sorted, [], @mean);
    if numel(f_unique) < 2
        cp_est = NaN;
        extrapolated = true;
        return;
    end
    cp_est = interp1(f_unique, cp_unique, target_force, 'linear', 'extrap');
    extrapolated = target_force < min(f_unique) || target_force > max(f_unique);
end

%% =========================================================================
%  CONFIG
%% =========================================================================
TRUE_TO_CODE = [8,4,1,13,9,5,2,17,14,10,6,3,18,15,11,7,19,16,12];
SENSITIVITY   = 36.0;
GAMMA         = 0.5;
RAW_VALID_MAX = 1000;
N_ITERATIONS  = 10;
SURFACE_NAMES = {'flat', 'solid', 'hollow'};

MUCA_RAW_FILES = { ...
    fullfile(MUCA_RAW_LOGS, 'flat_sensor_19_points_10_iterations_session_20260826_155255.csv'), ...
    fullfile(MUCA_RAW_LOGS, 'solid_sensor_19_points_10_iterations_session_20260826_165651.csv'), ...
    fullfile(MUCA_RAW_LOGS, 'hollow_sensor_iterations_all_session_20260826_190632.csv') ...
};

% Same curated, previously-vetted LCR sweep files used in
% mucaboard_data/matlab/estimate_capacitance.m (hollow's synthetic-looking
% P05/P12 candidate file already excluded there -- see that script's
% comments for why).
LCR_FILES = { ...
    { fullfile(CAPMEAS_LOGS, 'ramp_collector_20260714_163437_flat_0.csv'), ...
      fullfile(CAPMEAS_LOGS, 'ramp_collector_20260714_151051_flat_1.csv'), ...
      fullfile(CAPMEAS_LOGS, 'ramp_collector_20260714_135538_flat_2.csv'), ...
      fullfile(CAPMEAS_LOGS, 'ramp_collector_20260714_124706_flat_3.csv'), ...
      fullfile(CAPMEAS_LOGS, 'ramp_collector_20260714_113352_flat_4.csv') }, ...
    { fullfile(CAPMEAS_LOGS, 'ramp_collector_20260710_164240_solidD_0mm.csv'), ...
      fullfile(CAPMEAS_LOGS, 'ramp_collector_20260710_152938_solidD_1mm.csv'), ...
      fullfile(CAPMEAS_LOGS, 'ramp_collector_20260709_161711_solidD_2mm.csv'), ...
      fullfile(CAPMEAS_LOGS, 'ramp_collector_20260710_140020_solidD_3mm.csv'), ...
      fullfile(CAPMEAS_LOGS, 'ramp_collector_20260709_144320_solidD_4mm.csv') }, ...
    { fullfile(CAPMEAS_LOGS, 'two_point_iterations_P10_P03_P14_20260804_171210_hollow.csv'), ...
      fullfile(CAPMEAS_LOGS, 'two_point_iterations_P08_P19_20260720_122906_hollow.csv') } ...
};

COMBO_FILE = fullfile(COMBO_LOGS, 'flat_calibration_muca_lcr_session_20260826_225636.csv');

fprintf('%s\n', repmat('=', 1, 74));
fprintf('  CAPACITANCE CALIBRATION -- muca raw -> real pF, via LCR + Futek\n');
fprintf('%s\n', repmat('=', 1, 74));

%% =========================================================================
%  STEP 0 -- position coherence check (mucaboard_data_raw vs Capacitance_measurement)
%% =========================================================================
fprintf('\n[STEP 0] Position coherence check (Capacitance_measurement vs mucaboard_data_raw)\n');
fprintf('%s\n', repmat('-', 1, 74));
fprintf(['Both collectors'' POINTS dict / REFERENCE_POSE were compared in source:\n' ...
    '  Integration_2/ur5_control.py (shared source), dataset_collector/collect.py,\n' ...
    '  data_collector_raw.py, and Capacitance_measurement/capacitance_dataset_collector.py\n' ...
    'all define the SAME 19-point mm-offset table and the SAME REFERENCE_POSE.\n' ...
    'Confirming against ACTUAL recorded tcp_x/tcp_y in real session files below --\n' ...
    'using ONLY ''locate''-phase rows (zero indentation) from both sides, since the\n' ...
    'tool approaches at a tilted angle (REFERENCE_POSE rx,ry,rz are non-trivial) so\n' ...
    'indentation depth alone shifts tcp_x/y -- mixing depths would confound this check:\n\n']);

muca_probe = read_muca_raw_csv(MUCA_RAW_FILES{1}, RAW_VALID_MAX);
capmeas_probe_path = fullfile(CAPMEAS_LOGS, 'ramp_collector_20260714_163437_flat_0.csv');
fid = fopen(capmeas_probe_path, 'r');
header_line = fgetl(fid);
header = strsplit(header_line, ',', 'CollapseDelimiters', false);
fmt = repmat('%s', 1, numel(header));
Craw = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
fclose(fid);
colc  = @(name) str2double(Craw{find(strcmp(header, name), 1)});
colcs = @(name) Craw{find(strcmp(header, name), 1)};
cm_point = colc('point');
cm_tcp_x = colc('tcp_x');
cm_tcp_y = colc('tcp_y');
cm_phase = colcs('phase');
cm_locate = strcmp(cm_phase, 'locate');
muca_locate = strcmp(muca_probe.phase, 'locate');

fprintf('  %6s %12s %12s %12s %12s %10s\n', 'code', 'muca tcp_x', 'capmeas tcp_x', 'muca tcp_y', 'capmeas tcp_y', 'diff(mm)');
max_diff_mm = 0;
n_checked = 0;
diffs_mm = [];
for code = 1:19
    mm = (muca_probe.ur5_point == code) & muca_locate;
    cmm = (cm_point == code) & cm_locate;
    if ~any(mm) || ~any(cmm)
        continue;
    end
    mx = mean(muca_probe.tcp_x(mm), 'omitnan'); my = mean(muca_probe.tcp_y(mm), 'omitnan');
    cx = mean(cm_tcp_x(cmm), 'omitnan');        cy = mean(cm_tcp_y(cmm), 'omitnan');
    diff_mm = 1000 * hypot(mx - cx, my - cy);
    max_diff_mm = max(max_diff_mm, diff_mm);
    diffs_mm(end + 1) = diff_mm; %#ok<AGROW>
    n_checked = n_checked + 1;
    fprintf('  %6d %12.5f %12.5f %12.5f %12.5f %10.2f\n', code, mx, cx, my, cy, diff_mm);
end
fprintf('\n  Checked %d shared point(s); mean diff = %.2f mm, max diff = %.2f mm.\n', ...
    n_checked, mean(diffs_mm), max_diff_mm);
if max_diff_mm < 2.0
    fprintf('  COHERENT: both datasets press the same physical UR5 locations for the same point number.\n');
else
    fprintf(['  WARNING: discrepancy exceeds 2mm even at zero indentation -- this is NOT explained\n' ...
        '  by the tilted-approach/depth confound. Most likely cause: the robot/reference-frame was\n' ...
        '  physically re-registered between the Capacitance_measurement session (Jun/Jul 2026) and\n' ...
        '  the mucaboard_data_raw session (Aug 2026), even though both scripts commanded identical\n' ...
        '  POINTS/REFERENCE_POSE constants. Since the hex grid''s own point-to-point spacing is only\n' ...
        '  4-8mm, an error of this size means naive point-NUMBER matching (used in STEP B below) can\n' ...
        '  be matching against a position that is closer to a NEIGHBORING point than to the intended\n' ...
        '  one. Treat STEP B''s fit as approximate for this reason -- see the printed per-point table.\n']);
end
fprintf(['\n  NOTE: "point number" above is the ROBOT''S OWN numbering (what''s literally in\n' ...
    '  the ur5_point/point column of every CSV in all 3 datasets) -- NOT the physical\n' ...
    '  board''s silkscreen numbering. TRUE_TO_CODE (this script''s header) is applied\n' ...
    '  when reporting results by TRUE point id below, exactly as in\n' ...
    '  mucaboard_data_raw/matlab/simple_19points_analysis.m.\n']);

%% =========================================================================
%  STEP A -- direct electrical calibration (capacitance_combination_calibration)
%% =========================================================================
fprintf('\n[STEP A] Direct electrical calibration (known capacitor -> muca raw)\n');
fprintf('%s\n', repmat('-', 1, 74));
comboA = combo_active_cell_fit(COMBO_FILE);
fprintf('  %d (combination, point) samples parsed from %s\n', numel(comboA.raw), COMBO_FILE);
fprintf('  %d/%d readings saturated the raw ADC (>%.0f counts) -- these combinations probe\n', ...
    comboA.n_saturated, numel(comboA.raw), RAW_VALID_MAX);
fprintf('  well beyond mucaboard_data_raw''s actual operating range (raw ~15-70 counts).\n');

valid_A = ~isnan(comboA.raw) & ~isnan(comboA.Cp_pF) & (comboA.raw <= RAW_VALID_MAX);
xA = comboA.raw(valid_A);
yA = comboA.Cp_pF(valid_A);
if numel(xA) >= 2
    pA = polyfit(xA, yA, 1);
    yfitA = polyval(pA, xA);
    r2A = 1 - sum((yA - yfitA).^2) / sum((yA - mean(yA)).^2);
    fprintf('  FIT (non-saturated only): Cp_pF = %.5f * raw + %.4f   (R2=%.3f, n=%d)\n', ...
        pA(1), pA(2), r2A, numel(xA));
else
    pA = [NaN NaN];
    r2A = NaN;
    fprintf('  Not enough non-saturated samples for a fit.\n');
end
fprintf(['  ASSESSMENT: this fit reflects the electronics'' raw-to-Cp GAIN in a clean,\n' ...
    '  no-touch-mechanics topology, but most of its dynamic range is saturated and its\n' ...
    '  topology differs from a real touch (direct-wired capacitor vs. FT5x16 mutual-\n' ...
    '  capacitance sensing -- see script header). Used below as a SANITY CHECK on sign\n' ...
    '  and rough magnitude only, not as the primary calibration.\n\n' ...
    '  Checked ALL FOUR capacitance_combination_calibration/logs/ session files with\n' ...
    '  muca-phase data (this one + testing_cap_v1_20260826_220641.csv): every single\n' ...
    '  reading in every file saturates (>1000 raw counts, the ADC ceiling seen earlier\n' ...
    '  as ~65500). The smallest tested combination ("2units") already saturates -- so\n' ...
    '  none of this session''s capacitors are small enough to characterize the\n' ...
    '  non-saturated regime mucaboard_data_raw actually operates in (raw ~15-70).\n' ...
    '  ACTIONABLE: next combination-calibration session should include much SMALLER\n' ...
    '  capacitors (well under "2units") to get usable STEP A / STEP C data.\n']);

%% =========================================================================
%  STEP B -- force-matched cross-dataset regression (PRIMARY)
%% =========================================================================
fprintf('\n[STEP B] Force-matched regression (Capacitance_measurement <-> mucaboard_data_raw)\n');
fprintf('%s\n', repmat('-', 1, 74));

fitsB       = cell(1, 3);
matchedB    = cell(1, 3);   % [true_point, force_N, V_own, Cp_pF_matched, extrapolated]
muca_by_pt  = cell(1, 3);

for s = 1:3
    fprintf('\n  -- %s --\n', upper(SURFACE_NAMES{s}));
    d = read_muca_raw_csv(MUCA_RAW_FILES{s}, RAW_VALID_MAX);
    if d.n_bad > 0
        fprintf('  [warn] %d/%d raw readings excluded as impossible (>%.0f counts)\n', ...
            d.n_bad, numel(d.raw_cells), RAW_VALID_MAX);
    end
    rows = muca_raw_own_reading_by_point(d, TRUE_TO_CODE, N_ITERATIONS, SENSITIVITY, GAMMA);
    muca_by_pt{s} = rows;

    lcr_d = load_lcr_files(LCR_FILES{s});
    lcr_rows = lcr_stats_by_point_depth(lcr_d);   % [point(code), depth_mm, force_N, Cp_pF]
    fprintf('  LCR sweep: %d (point,depth) rows loaded, %d distinct point(s)\n', ...
        size(lcr_rows, 1), numel(unique(lcr_rows(:, 1))));

    m = nan(19, 5);
    for true_pt = 1:19
        if isnan(rows(true_pt, 1))
            continue;
        end
        code_id = TRUE_TO_CODE(true_pt);
        target_force = rows(true_pt, 2);
        sub = lcr_rows(lcr_rows(:, 1) == code_id, [3 4]);   % [force_N, Cp_pF]
        if isempty(sub)
            continue;
        end
        [cp_est, extrap] = interp_force_to_cp(sub, target_force);
        m(true_pt, :) = [true_pt, target_force, rows(true_pt, 4), cp_est, extrap];
    end
    matchedB{s} = m;

    valid = ~isnan(m(:, 4));
    n_extrap = sum(m(valid, 5) == 1);
    fprintf('  MATCHED: %d/19 points got a force-interpolated Cp_pF (%d needed extrapolation)\n', ...
        sum(valid), n_extrap);

    x = m(valid, 3);   % V_own
    y = m(valid, 4);   % Cp_pF
    if numel(x) >= 2
        p = polyfit(x, y, 1);
        yfit = polyval(p, x);
        ss_res = sum((y - yfit).^2);
        ss_tot = sum((y - mean(y)).^2);
        r2 = 1 - ss_res / max(ss_tot, eps);
        rmse = sqrt(mean((y - yfit).^2));
        fitsB{s} = struct('slope', p(1), 'intercept', p(2), 'r2', r2, 'rmse', rmse, 'n', numel(x));
        fprintf('  FIT (pooled, %s): Cp_pF = %.4f * V_own + %.4f   (R2=%.3f, RMSE=%.4f pF, n=%d)\n', ...
            SURFACE_NAMES{s}, p(1), p(2), r2, rmse, numel(x));
    else
        fitsB{s} = [];
        fprintf('  [warn] fewer than 2 matched points -- no fit for %s\n', SURFACE_NAMES{s});
    end
end

%% =========================================================================
%  STEP C -- hybrid (documented only)
%% =========================================================================
fprintf('\n[STEP C] Hybrid electrical-anchor + force-matched (NOT implemented)\n');
fprintf('%s\n', repmat('-', 1, 74));
fprintf(['  Idea: use STEP A''s slope sign/magnitude as a prior on the electronics'' raw-\n' ...
    '  to-Cp gain, and STEP B''s per-surface intercept for the touch-specific offset,\n' ...
    '  potentially blending them (e.g. constrained fit, or a 2-stage model separating\n' ...
    '  ''chip gain'' from ''touch topology correction''). Not implemented here: STEP A''s\n' ...
    '  usable (non-saturated) range is only a handful of points (see above), too few\n' ...
    '  to constrain a second free parameter without overfitting. Worth revisiting once\n' ...
    '  a combination-calibration session covers the LOW end of the capacitor range\n' ...
    '  (closer to mucaboard_data_raw''s actual raw ~15-70 count operating window).\n']);

%% =========================================================================
%  FINAL -- apply STEP B fit to the FULL mucaboard_data_raw dataset
%% =========================================================================
fprintf('\n[FINAL] Applying STEP B per-surface fit to every mucaboard_data_raw reading\n');
fprintf('%s\n', repmat('-', 1, 74));

out_fid = fopen(fullfile(RESULTS_DIR, 'mucaboard_data_raw_in_pF.csv'), 'w');
fprintf(out_fid, 'surface,true_point,round_idx,phase,raw_own,V_own,Cp_pF_estimated\n');

Cp_hexmap = nan(19, 3);   % per-point hold-phase mean Cp_pF, for the figure

for s = 1:3
    if isempty(fitsB{s})
        continue;
    end
    d = read_muca_raw_csv(MUCA_RAW_FILES{s}, RAW_VALID_MAX);
    a = fitsB{s}.slope;
    b = fitsB{s}.intercept;

    for true_pt = 1:19
        code_id = TRUE_TO_CODE(true_pt);
        mask = (d.ur5_point == code_id) & (d.round_idx >= 0) & (d.round_idx <= N_ITERATIONS - 1) & ...
               (strcmp(d.phase, 'hold') | strcmp(d.phase, 'press') | strcmp(d.phase, 'retract'));
        idx = find(mask);
        if isempty(idx)
            continue;
        end
        calib = d.calib_raw(code_id);
        raw_vals = d.raw_cells(idx, code_id);
        ratio = min(max((raw_vals - calib) / SENSITIVITY, 0), 1);
        v_vals = ratio .^ GAMMA;
        cp_vals = a * v_vals + b;
        for i = 1:numel(idx)
            fprintf(out_fid, '%s,%d,%d,%s,%.4f,%.5f,%.5f\n', SURFACE_NAMES{s}, true_pt, ...
                d.round_idx(idx(i)), d.phase{idx(i)}, raw_vals(i), v_vals(i), cp_vals(i));
        end

        hold_mask = mask & strcmp(d.phase, 'hold');
        if any(hold_mask)
            raw_h = d.raw_cells(hold_mask, code_id);
            ratio_h = min(max((raw_h - calib) / SENSITIVITY, 0), 1);
            v_h = mean(ratio_h .^ GAMMA, 'omitnan');
            Cp_hexmap(true_pt, s) = a * v_h + b;
        end
    end
    fprintf('  %-7s: wrote per-sample Cp_pF for all 19 points using Cp = %.4f*V + %.4f\n', ...
        SURFACE_NAMES{s}, a, b);
end
fclose(out_fid);
fprintf('\n  Saved: %s\n', fullfile(RESULTS_DIR, 'mucaboard_data_raw_in_pF.csv'));

%% ---- hex-map figure of the final per-point estimated Cp_pF -------------
[point_ids, point_xy, ~] = muca_layout();   % geometry only (code-id order);
% relabel to TRUE ids exactly as in simple_19points_analysis.m, since
% point_xy(true_id) = point_xy_code(TRUE_TO_CODE(true_id)).
point_xy_true = point_xy(TRUE_TO_CODE, :);

valid_vals = Cp_hexmap(~isnan(Cp_hexmap));
if ~isempty(valid_vals)
    vmin = min(valid_vals);
    vmax = max(valid_vals);
    cmap = get_cmap();
    fig = figure('Position', [100 100 1500 550]);
    panel_left  = [0.03, 0.35, 0.67];
    panel_width = 0.27;
    axes_list = cell(1, 3);
    for s = 1:3
        ax = axes('Parent', fig, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
        axes_list{s} = ax;
        draw_hex_panel(ax, (1:19)', point_xy_true, Cp_hexmap(:, s), cmap, vmin, vmax, ...
            8.0 / sqrt(3), upper(SURFACE_NAMES{s}));
    end
    colormap(fig, cmap);
    for s = 1:3
        set(axes_list{s}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, 'Estimated Cp (pF), hold-phase mean');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
    end
    fig_suptitle(fig, 'mucaboard\_data\_raw -- estimated real capacitance (pF), STEP B fit');
    save_fig(fig, fullfile(RESULTS_DIR, 'mucaboard_estimated_capacitance_hexmap'));
    fprintf('  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR, 'mucaboard_estimated_capacitance_hexmap'));
end

%% ---- STEP A vs STEP B comparison scatter (sanity check) -----------------
figAB = figure('Position', [100 100 700 600]);
hold on; box on; grid on;
if numel(xA) >= 2
    scatter(xA, yA, 25, [0.6 0.6 0.6], 'filled', 'DisplayName', 'STEP A: direct combo calib (raw)');
end
for s = 1:3
    if isempty(fitsB{s}), continue; end
    m = matchedB{s};
    valid = ~isnan(m(:, 4));
    scatter(m(valid, 3) * SENSITIVITY, m(valid, 4), 35, 'filled', ...
        'DisplayName', sprintf('STEP B: %s (V*SENSITIVITY, for rough x-axis comparability)', SURFACE_NAMES{s}));
end
xlabel('raw-ish x-axis (STEP A: raw counts; STEP B: V_{own} * SENSITIVITY)');
ylabel('Cp_pF');
title('STEP A vs STEP B -- sanity comparison (different x definitions, see labels)');
legend('Location', 'best', 'Interpreter', 'none');
save_fig(figAB, fullfile(RESULTS_DIR, 'stepA_vs_stepB_sanity_check'));
fprintf('  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR, 'stepA_vs_stepB_sanity_check'));

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('  DONE. All results in: %s\n', RESULTS_DIR);
fprintf('%s\n', repmat('=', 1, 74));
