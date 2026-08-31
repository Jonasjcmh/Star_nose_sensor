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

%% ---- shared helpers now live in their own files in this folder --------
% (read_muca_raw_csv.m, muca_raw_own_reading_by_point.m,
% combo_active_cell_fit.m, interp_force_to_cp.m) -- factored out so
% per_point_surface_relationships.m can reuse them without duplication.
% Octave/MATLAB pick these up automatically since they sit next to this
% script (both are run from this same directory).

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

% Full 19-point LCR sweeps, one file per surface (depths 5-10mm, so
% mucaboard_data_raw's actual press depth -- 10mm -- is covered directly,
% no depth-offset hack needed like the old partial hollow-only sweep).
% Supersedes the old curated multi-file lists (flat/solid: 5 ramp_collector
% files each, hollow: only 5/19 points via two_point_iterations_P10_P03_P14
% + P08_P19) -- those covered far fewer points, especially hollow.
LCR_FILES = { ...
    { fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260828_193051_flat_sensor.csv') }, ...
    { fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260828_162718_solid_sensor.csv') }, ...
    { fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260825_202746_hollow_sensor.csv') } ...
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
capmeas_probe_path = fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260828_193051_flat_sensor.csv');
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
if max_diff_mm < 3.0
    fprintf(['  COHERENT: both datasets press the same physical UR5 locations for the same point\n' ...
        '  number, well within the hex grid''s own 4-8mm point spacing -- residual mm-scale noise\n' ...
        '  is expected from independent robot moves, not a registration problem. (This LCR sweep,\n' ...
        '  two_point_iterations_ALL19_*, was collected 2026-08-25/28, close in time to\n' ...
        '  mucaboard_data_raw''s 2026-08-26 session -- unlike the older Jun/Jul LCR files, which\n' ...
        '  showed a much larger ~11mm mean / 17mm max discrepancy here, consistent with the robot\n' ...
        '  having been re-registered at some point between June/July and August.)\n']);
else
    fprintf(['  WARNING: discrepancy exceeds the hex grid''s own 4-8mm point spacing even at zero\n' ...
        '  indentation -- naive point-NUMBER matching (used in STEP B below) can be matching against\n' ...
        '  a position closer to a NEIGHBORING point than the intended one. Treat STEP B''s fit as\n' ...
        '  approximate for this reason -- see the printed per-point table.\n']);
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
