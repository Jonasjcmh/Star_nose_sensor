%% a1_fit_lc_ur_calibration.m
%
% MATLAB port of fit_lc_ur_calibration.py.
%
% What this script does, step by step:
%   1. Finds the calibration CSV logs in ../logs/. Filenames may carry a
%      version tag (fzcal_..._v2_..., v3, ...); an un-tagged file is
%      implicitly v1. Per instrument, only the HIGHEST version number
%      present is trusted -- older batches are discarded wholesale, not
%      merged weight-by-weight, since a whole batch shares one day's
%      session-to-session baseline drift that isn't comparable across
%      batches (see keep_latest_version). Within that latest batch, if
%      the same weight was measured more than once, keeps only the most
%      recent attempt ("de-duplication"). No hand-maintained weight
%      list and no hardcoded version number to edit when new sessions
%      are collected -- print_dataset_manifest always prints exactly
%      which files were selected.
%   2. Loads each kept file (fz for both instruments; ai0 too for
%      futek_direct).
%   3. STEP 1 -- fits how the load cell's voltage changes with applied
%      force (its "change rate" / sensitivity, in N/V and V/N).
%   4. STEP 2/3 -- pairs, from the SAME session/phase, the load cell's
%      force estimate with the UR robot's fz reading, and fits the
%      compensation line the UR sensor needs. Fit twice: once on raw fz
%      (mixes push/pull, pooled R^2 ~0.1, only per-direction fits are
%      usable), and once on SIGN-CORRECTED fz -- fz_signed =
%      AI0_SIGN[direction] * |fz_raw|, which forces the UR reading onto
%      the same push/pull convention the load cell already has -- giving
%      ONE pooled line that actually fits (R^2 ~0.9):
%          F_loadcell = comp_slope_s * fz_signed + comp_offset_s
%      so a corrected reading can be computed as
%          fz_corrected = comp_slope_s * (AI0_SIGN[direction]*abs(fz_raw)) + comp_offset_s
%      This sign-corrected pooled fit is the one saved to the JSON output
%      and used for the Bland-Altman diagnostic; the raw-fz fit and the
%      per-direction fits are kept alongside it for reference.
%   5. STEP 4 -- an independent cross-check: the same fit, but using the
%      sessions where no load cell was installed (fzcal_ur_only_*), so
%      "truth" there is the known placed weight instead of the load cell.
%   6. Saves the fitted numbers to a JSON file and plots one figure.
%
% Baseline is a known load, not a zero reference (both instruments)
% --------------------------------------------------------------------
% For BOTH futek_direct and ur_only, the hardware (load cell + holder/hook
% for futek_direct; attachment + screws + holder/hook for ur_only) is
% already resting on the sensor during the no-load baseline (loaded==0)
% too -- it isn't removed between the baseline and loaded phases. So the
% baseline is a known, non-zero load in its own right, not something to
% zero out. Each de-duplicated session therefore contributes TWO absolute
% points (baseline, loaded), not one baseline-compensated delta:
%   futek_direct:  (ai0_base_mean, F_signed_base = hardware only)
%                  (ai0_load_mean, F_signed      = hardware + weight)
%   ur_only:       (fz_base_mean,  F_true_base   = hardware only)
%                  (fz_load_mean,  F_true        = hardware + weight)
% (A pure delta would have cancelled the hardware term exactly, since it's
% identical in both phases -- so adding hardware mass only matters, and
% only makes sense, once the baseline is treated this way instead of
% subtracted away.)
%
% Ground truth
% ------------
%     F_true = (total_g / 1000) * g * cos(tilt_from_vertical)
% where total_g is the nominal placed weight plus the hardware mass:
%   - fzcal_ur_only_*: total_g = weight_g + 43g (posz) / 37g (negz).
%   - fzcal_futek_direct_* (the load cell's own reading): total_g = weight_g
%     + 7g (posz, holder) / 4g (negz, hook) -- only what's mounted ABOVE
%     the load cell in the load path, not its own body or the UR mount
%     below it.
% Both hardware masses were confirmed by whoever ran the collection.
%
% Run this file directly (F5, or "run a1_fit_lc_ur_calibration" from the
% force_sensor_calibration/matlab folder). No toolboxes required.

clear; clc; close all;

% All plots in this script: Helvetica 10pt (falls back to the system
% default if Helvetica isn't installed).
set(groot, 'defaultAxesFontName', 'Helvetica', 'defaultAxesFontSize', 10, ...
           'defaultTextFontName', 'Helvetica', 'defaultTextFontSize', 10, ...
           'defaultLegendFontName', 'Helvetica', 'defaultLegendFontSize', 10);

%% ---- Paths & constants ----

HERE = fileparts(mfilename('fullpath'));          % .../force_sensor_calibration/matlab
CALIB_DIR = fileparts(HERE);                        % .../force_sensor_calibration
LOG_DIR = fullfile(CALIB_DIR, 'logs');
OUT_DIR = fullfile(CALIB_DIR, 'plots');
if ~exist(OUT_DIR, 'dir')
    mkdir(OUT_DIR);
end

G = 9.80665;                                 % standard gravity, m/s^2

% Toggle: show the "{real_g}g->{F_signed}N" text label next to each
% loaded point in Figure 1. Off by default (cleaner plot); flip to true
% to bring the per-point annotations back.
SHOW_POINT_LABELS = false;

%% ---- Step 0: discover + de-duplicate both instruments' sessions ----
% No hand-maintained weight list and no hardcoded version number here --
% see discover_entries and keep_latest_version below for how new
% re-collections (new weights, new "v3"/"v4" batches, ...) get picked up
% automatically. print_dataset_manifest always prints exactly which
% files were selected, so this is also the "easy way to check" the
% datasets in use.

futek_entries = discover_entries(LOG_DIR, 'futek_direct');
futek_entries = keep_latest_version(futek_entries);
futek_entries = dedupe_latest(futek_entries);
print_dataset_manifest(futek_entries, 'futek_direct');

ur_entries = discover_entries(LOG_DIR, 'ur_only');
ur_entries = keep_latest_version(ur_entries);
ur_entries = dedupe_latest(ur_entries);
print_dataset_manifest(ur_entries, 'ur_only');

%% ---- Step 0b: load every kept file ----

futek_sessions_cell = cell(1, numel(futek_entries));
for i = 1:numel(futek_entries)
    futek_sessions_cell{i} = load_session(futek_entries(i), G);
end
futek_sessions = sort_sessions([futek_sessions_cell{:}]);

ur_sessions_cell = cell(1, numel(ur_entries));
for i = 1:numel(ur_entries)
    ur_sessions_cell{i} = load_session(ur_entries(i), G);
end
ur_sessions = sort_sessions([ur_sessions_cell{:}]);

% Expand futek_direct sessions into 2 absolute points each (baseline,
% loaded) -- see header comment for why the baseline isn't a zero ref.
points = expand_phases(futek_sessions);
ai0_pts = [points.ai0]';
fsigned_pts = [points.F_signed]';
fz_pts = [points.fz]';
fz_signed_pts = [points.fz_signed]';
is_posz_pts = strcmp({points.direction}, 'posz')';
is_negz_pts = strcmp({points.direction}, 'negz')';



%% ---- Step 1: load-cell voltage <-> force change rate ----

[lc_rate_N_per_V, lc_offset_N, lc_r2, lc_rmse] = linfit(ai0_pts, fsigned_pts);

fprintf('%s\n', repmat('=', 1, 78));
fprintf('STEP 1 -- FUTEK load cell: voltage <-> force change rate\n');
fprintf('baseline (hardware only) and loaded (hardware+weight) each a real, known point\n');
fprintf('%s\n', repmat('=', 1, 78));
fprintf('%8s%6s%9s%8s%10s%15s%11s%10s\n', 'weight_g', 'dir', 'phase', 'real_g', 'ai0(V)', ...
        'F_expected(N)', 'F_pred(N)', 'resid(N)');
for i = 1:numel(points)
    p = points(i);
    f_pred = lc_rate_N_per_V * p.ai0 + lc_offset_N;
    fprintf('%8.0f%6s%9s%8.0f%10.4f%+15.4f%+11.4f%+10.4f\n', p.weight_g, p.direction, p.phase, ...
            p.compensated_g, p.ai0, p.F_signed, f_pred, f_pred - p.F_signed);
end
fprintf('  real_g = nominal weight + hardware mass (holder/hook, 7g posz / 4g negz)\n');
fprintf('  F_expected(N) = F_signed, the ground truth force from real_g (what SHOULD be measured)\n');
fprintf('  F_pred(N) = the fit''s own estimate from this point''s ai0 voltage\n');
fprintf('\nn = %d points (%d sessions x 2 phases each)\n', numel(points), numel(futek_sessions));
fprintf('F_signed = %.4f * ai0 + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        lc_rate_N_per_V, lc_offset_N, lc_r2, lc_rmse);
fprintf('  -> load-cell sensitivity: %.4f N/V (equivalently %.5f V/N change rate)\n', ...
        lc_rate_N_per_V, 1 / lc_rate_N_per_V);

%% ---- Step 2/3: UR fz vs load-cell force (same session/phase pairing) ----
% F_lc per point is computed from THAT SAME phase's own ai0, through the
% Step-1 fit -- so the UR sensor is corrected against what the load cell
% actually reported, not against the known weight.

flc_pts = lc_rate_N_per_V * ai0_pts + lc_offset_N;
[comp_slope, comp_offset, comp_r2, comp_rmse] = linfit(fz_pts, flc_pts);

fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('STEP 2/3 -- F_lc vs UR fz (same session/phase) + UR compensation coefficients\n');
fprintf('%s\n', repmat('=', 1, 78));
fprintf('%8s%6s%9s%10s%10s\n', 'weight_g', 'dir', 'phase', 'fz(N)', 'F_lc(N)');
for i = 1:numel(points)
    p = points(i);
    fprintf('%8.0f%6s%9s%10.4f%10.4f\n', p.weight_g, p.direction, p.phase, fz_pts(i), flc_pts(i));
end
fprintf('\nn = %d points (%d sessions x 2 phases each)\n', numel(points), numel(futek_sessions));
fprintf('F_lc = %.4f * fz_robot + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        comp_slope, comp_offset, comp_r2, comp_rmse);
fprintf('  -> apply as: fz_corrected = %.4f * fz_raw + (%.5f)\n', comp_slope, comp_offset);

%% ---- Step 2/3b: does ONE pooled correction actually make sense? ----
% Fit F_lc = a*fz_robot + b separately for posz and negz. A single pooled
% (a, b) only makes sense if the two directions actually agree -- this
% checks that assumption instead of assuming it.

[posz_slope, posz_offset, posz_r2, posz_rmse] = linfit(fz_pts(is_posz_pts), flc_pts(is_posz_pts));
[negz_slope, negz_offset, negz_r2, negz_rmse] = linfit(fz_pts(is_negz_pts), flc_pts(is_negz_pts));

fprintf('\nPer-direction compensation (same F_lc = a*fz + b form, fit separately):\n');
fprintf('  posz: F_lc = %.4f * fz_robot + (%.5f)   R^2 = %.5f   RMSE = %.4f N   (n=%d points)\n', ...
        posz_slope, posz_offset, posz_r2, posz_rmse, sum(is_posz_pts));
fprintf('  negz: F_lc = %.4f * fz_robot + (%.5f)   R^2 = %.5f   RMSE = %.4f N   (n=%d points)\n', ...
        negz_slope, negz_offset, negz_r2, negz_rmse, sum(is_negz_pts));
fprintf('  -> pooling hides a %.3f slope gap and %.3f N offset gap between directions.\n', ...
        abs(posz_slope - negz_slope), abs(posz_offset - negz_offset));

%% ---- Step 2/3c: sign-corrected fz -- same push/pull directionality as the load cell ----
% fz_signed = AI0_SIGN[direction] * |fz_raw|, so a SINGLE pooled
% correction is meaningful instead of the two disagreeing per-direction
% lines above.

[comp_slope_s, comp_offset_s, comp_r2_s, comp_rmse_s] = linfit(fz_signed_pts, flc_pts);

fprintf(['\nSign-corrected UR compensation (fz_signed = AI0_SIGN[direction] * |fz_raw|, ' ...
         'matching the load cell''s own push/pull convention):\n']);
fprintf('  F_lc = %.4f * fz_signed + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        comp_slope_s, comp_offset_s, comp_r2_s, comp_rmse_s);
fprintf(['  -> apply as: fz_corrected = %.4f * (AI0_SIGN[direction] * |fz_raw|) + (%.5f), ' ...
         'one formula for both directions (R^2 %.4f vs %.4f pooled-raw)\n'], ...
        comp_slope_s, comp_offset_s, comp_r2_s, comp_r2);

%% ---- Bland-Altman: where does the DEPLOYABLE (sign-corrected pooled) correction agree? ----
% Using the raw pooled fit here would just restate how badly
% pooling-without-signing fails (R^2 ~0.1) -- now that a genuine single
% pooled correction exists, that's the one worth checking agreement
% bounds for.

fz_corrected_pooled = comp_slope_s * fz_signed_pts + comp_offset_s;
ba_diff = fz_corrected_pooled - flc_pts;
ba_mean_pair = (fz_corrected_pooled + flc_pts) / 2;
ba_bias = mean(ba_diff);
ba_sd = std(ba_diff);
ba_lo = ba_bias - 1.96 * ba_sd;
ba_hi = ba_bias + 1.96 * ba_sd;

fprintf('\nBland-Altman diagnostic (sign-corrected pooled correction: fz_corrected vs F_lc):\n');
fprintf('  overall bias = %+.4f N   limits of agreement = [%+.4f, %+.4f] N\n', ba_bias, ba_lo, ba_hi);
fprintf('  posz bias = %+.4f N (std %.4f N, n=%d)\n', ...
        mean(ba_diff(is_posz_pts)), std(ba_diff(is_posz_pts)), sum(is_posz_pts));
fprintf('  negz bias = %+.4f N (std %.4f N, n=%d)\n', ...
        mean(ba_diff(is_negz_pts)), std(ba_diff(is_negz_pts)), sum(is_negz_pts));

%% ---- Step 4: ur_only cross-check (no load cell, vs known weight) ----

fz_ur_only = reshape([[ur_sessions.fz_base_mean]; [ur_sessions.fz_load_mean]], [], 1);
F_true_ur_only = reshape([[ur_sessions.F_true_base]; [ur_sessions.F_true]], [], 1);
[crosscheck_slope, crosscheck_offset, crosscheck_r2, crosscheck_rmse] = ...
    linfit(fz_ur_only, F_true_ur_only);

fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('STEP 4 -- ur_only cross-check (no load cell, vs known weight directly)\n');
fprintf('baseline (hardware only) and loaded (hardware+weight) each a real, known point\n');
fprintf('%s\n', repmat('=', 1, 78));
fprintf('%8s%6s%9s%11s%11s\n', 'weight_g', 'dir', 'phase', 'fz_abs(N)', 'F_true(N)');
for i = 1:numel(ur_sessions)
    s = ur_sessions(i);
    fprintf('%8.0f%6s%9s%11.4f%11.4f\n', s.weight_g, s.direction, 'baseline', ...
            s.fz_base_mean, s.F_true_base);
    fprintf('%8.0f%6s%9s%11.4f%11.4f\n', s.weight_g, s.direction, 'loaded', ...
            s.fz_load_mean, s.F_true);
end
fprintf('\nn = %d points (%d sessions x 2 phases each)\n', numel(fz_ur_only), numel(ur_sessions));
fprintf('F_true = %.4f * fz_robot + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        crosscheck_slope, crosscheck_offset, crosscheck_r2, crosscheck_rmse);
fprintf('  compare vs Step 3: comp_slope=%.4f vs crosscheck_slope=%.4f (%.1f%% apart), ', ...
        comp_slope, crosscheck_slope, abs(comp_slope - crosscheck_slope) / abs(comp_slope) * 100);
fprintf('comp_offset=%.4f vs crosscheck_offset=%.4f\n', comp_offset, crosscheck_offset);
fprintf('%s\n', repmat('=', 1, 78));

%% ---- Save compensation coefficients (same fields as the Python SOP JSON) ----

% Top-level slope/offset is the SIGN-CORRECTED pooled fit -- it's the one
% deployable formula (fz_signed = AI0_SIGN[direction]*|fz_raw|); the raw
% pooled fit is kept alongside it for reference/diagnostics only (it mixes
% push and pull, R^2 ~0.1).
calib_out = struct( ...
    'tip', 'futek_direct', ...
    'date', char(datetime('today', 'Format', 'yyyy-MM-dd')), ...
    'slope', comp_slope_s, ...
    'offset', comp_offset_s, ...
    'r_squared', comp_r2_s, ...
    'rmse_n', comp_rmse_s, ...
    'fz_input', ['fz_signed = AI0_SIGN[direction] * abs(fz_raw); ' ...
                 'AI0_SIGN = {posz: -1, negz: +1}'], ...
    'n_samples', numel(points), ...
    'n_sessions', numel(futek_sessions), ...
    'loadcell_rate_n_per_v', lc_rate_N_per_V, ...
    'loadcell_rate_v_per_n', 1 / lc_rate_N_per_V, ...
    'note', ['F_true includes hardware mass for both instruments: ur_only uses 43g posz / ' ...
             '37g negz; futek_direct uses 7g posz / 4g negz. Baseline is treated as a ' ...
             'known non-zero load (hardware only), not a zero reference -- n_samples = ' ...
             'sessions x 2 (baseline + loaded).'], ...
    'raw_unsigned_pooled', struct( ...
        'slope', comp_slope, 'offset', comp_offset, 'r_squared', comp_r2, 'rmse_n', comp_rmse, ...
        'note', ['fit on raw fz (no sign correction) -- pools push and pull into one ' ...
                 'line and fits neither well; kept for reference.']), ...
    'cross_check', struct( ...
        'tip', 'ur_only', 'slope', crosscheck_slope, 'offset', crosscheck_offset, ...
        'r_squared', crosscheck_r2, 'rmse_n', crosscheck_rmse, ...
        'n_samples', 2 * numel(ur_sessions), 'n_sessions', numel(ur_sessions)), ...
    'per_direction', struct( ...
        'posz', struct('slope', posz_slope, 'offset', posz_offset, 'r2', posz_r2, ...
                        'rmse', posz_rmse, 'n', sum(is_posz_pts)), ...
        'negz', struct('slope', negz_slope, 'offset', negz_offset, 'r2', negz_r2, ...
                        'rmse', negz_rmse, 'n', sum(is_negz_pts))), ...
    'bland_altman_pooled', struct( ...
        'note', 'sign-corrected pooled correction vs F_lc', ...
        'bias_n', ba_bias, 'loa_lower_n', ba_lo, 'loa_upper_n', ba_hi) ...
);

calib_path = fullfile(CALIB_DIR, 'calib_fz_lc_pattern_matlab.json');
try
    json_text = jsonencode(calib_out, 'PrettyPrint', true);   % needs R2021a+
catch
    json_text = jsonencode(calib_out);                          % older MATLAB: compact JSON
end
fid = fopen(calib_path, 'w');
fprintf(fid, '%s', json_text);
fclose(fid);
fprintf('\nSaved compensation coefficients -> %s\n', calib_path);

%% ---- Plot: 4 separate figures, one per step, instead of one 2x2 grid ----

% --- Figure 1: load-cell voltage vs force (absolute) ---
fig1 = figure('Color', 'w', 'Position', [100 100 800 650]);
ax1 = axes(fig1);
hold(ax1, 'on');
is_loaded_pts = strcmp({points.phase}, 'loaded');
loaded_idx = find(is_loaded_pts);
for i = loaded_idx
    [color, marker, faceColor, sz, lw] = point_style(points(i));
    plot(ax1, ai0_pts(i), fsigned_pts(i), marker, 'Color', color, 'MarkerFaceColor', faceColor, ...
         'MarkerSize', sz, 'LineWidth', lw);
end
% fit itself still uses ALL points (baseline + loaded) -- baseline is a
% known, non-zero point, not something to discard from the fit, only
% from this display.
margin = 0.05 * (max(ai0_pts) - min(ai0_pts));
ai0_range = linspace(min(ai0_pts) - margin, max(ai0_pts) + margin, 200);
plot(ax1, ai0_range, lc_rate_N_per_V * ai0_range + lc_offset_N, 'k-', 'LineWidth', 2);
yline(ax1, 0, 'Color', [0.5 0.5 0.5]);

% Annotate the loaded points with the hardware-compensated real weight
% (nominal + hardware) -- the expected total each point represents, not
% just the nominal weight_g -- and the resulting F_signed value it
% should measure. Toggle via SHOW_POINT_LABELS.
if SHOW_POINT_LABELS
    dx = 0.01 * (max(ai0_pts) - min(ai0_pts));
    dy = 0.02 * (max(fsigned_pts) - min(fsigned_pts));
    for i = loaded_idx
        p = points(i);
        [color, ~, ~, ~, ~] = point_style(p);
        text(ax1, ai0_pts(i) + dx, fsigned_pts(i) + dy, ...
             sprintf('%.0fg->%+.2fN', p.compensated_g, p.F_signed), ...
             'FontSize', 10, 'Color', color);
    end
end

xlabel(ax1, 'ai_{0} (V)','FontName','Helvetica','FontWeight','bold');
ylabel(ax1, 'F (N)','FontName','Helvetica','FontWeight','bold');
%title(ax1, sprintf(['Step 1 -- load-cell voltage vs force (absolute)\n' ...
%      'F = %.3f*ai0 + %.3f (R^2=%.4f)  |  annotations: real weight (nominal+hardware) -> expected force'], ...
%      lc_rate_N_per_V, lc_offset_N, lc_r2));
grid(ax1, 'off');
%sgtitle(fig1, 'LC <-> UR calibration fit -- Step 1: load-cell voltage vs force');
out1_path = fullfile(OUT_DIR, 'lc_ur_calibration_step1_voltage_force_matlab.png');
print(fig1, out1_path, '-dpng', '-r150');
fprintf('Saved -> %s\n', out1_path);

% --- Figure 2: UR compensation vs load cell, raw fz (absolute) ---
fig2 = figure('Color', 'w', 'Position', [100 100 800 650]);
ax2 = axes(fig2);
hold(ax2, 'on');
for i = 1:numel(points)
    [color, marker, faceColor, sz, lw] = point_style(points(i));
    plot(ax2, fz_pts(i), flc_pts(i), marker, 'Color', color, 'MarkerFaceColor', faceColor, ...
         'MarkerSize', sz, 'LineWidth', lw);
end
margin = 0.05 * (max(fz_pts) - min(fz_pts));
fz_range = linspace(min(fz_pts) - margin, max(fz_pts) + margin, 200);
plot(ax2, fz_range, comp_slope * fz_range + comp_offset, 'k-', 'LineWidth', 2);
plot(ax2, fz_range, posz_slope * fz_range + posz_offset, ':', 'Color', [0.12 0.47 0.71], 'LineWidth', 1.8);
plot(ax2, fz_range, negz_slope * fz_range + negz_offset, ':', 'Color', [0.84 0.15 0.16], 'LineWidth', 1.8);
yline(ax2, 0, 'Color', [0.5 0.5 0.5]);
xline(ax2, 0, 'Color', [0.5 0.5 0.5]);
xlabel(ax2, 'fz_{ur}, absolute (N)   [filled=loaded, open=baseline]');
ylabel(ax2, 'F_{lc} (N)   [futek\_direct, same-session/phase paired]');
title(ax2, sprintf(['Step 2/3 -- UR compensation vs load cell, raw fz (absolute)\n' ...
      'pooled: F=%.3f*fz+%.3f (R^2=%.4f)  |  posz: R^2=%.4f  |  negz: R^2=%.4f'], ...
      comp_slope, comp_offset, comp_r2, posz_r2, negz_r2));
grid(ax2, 'off');
sgtitle(fig2, 'LC <-> UR calibration fit -- Step 2/3: UR compensation vs load cell, raw fz');
out2_path = fullfile(OUT_DIR, 'lc_ur_calibration_step2_raw_fz_matlab.png');
print(fig2, out2_path, '-dpng', '-r150');
fprintf('Saved -> %s\n', out2_path);

% --- Figure 3: sign-corrected fz -- same push/pull directionality as the
% load cell, so ONE pooled line actually fits (vs the X-crossing raw
% figure above) ---
fig3 = figure('Color', 'w', 'Position', [100 100 800 650]);
ax2s = axes(fig3);
hold(ax2s, 'on');
for i = 1:numel(points)
    [color, marker, faceColor, sz, lw] = point_style(points(i));
    plot(ax2s, fz_signed_pts(i), flc_pts(i), marker, 'Color', color, 'MarkerFaceColor', faceColor, ...
         'MarkerSize', sz, 'LineWidth', lw);
end
margin = 0.05 * (max(fz_signed_pts) - min(fz_signed_pts));
fz_signed_range = linspace(min(fz_signed_pts) - margin, max(fz_signed_pts) + margin, 200);
plot(ax2s, fz_signed_range, comp_slope_s * fz_signed_range + comp_offset_s, 'k-', 'LineWidth', 2);
yline(ax2s, 0, 'Color', [0.5 0.5 0.5]);
xline(ax2s, 0, 'Color', [0.5 0.5 0.5]);
xlabel(ax2s, 'fz_{signed} = AI0\_SIGN[direction]*|fz_{ur}| (N)   [filled=loaded, open=baseline]');
ylabel(ax2s, 'F_{lc} (N)   [futek\_direct, same-session/phase paired]');
title(ax2s, sprintf(['Step 2/3 -- UR compensation vs load cell, sign-corrected fz\n' ...
      '(same push/pull directionality as load cell)  pooled: F=%.3f*fz\\_signed+%.3f (R^2=%.4f)'], ...
      comp_slope_s, comp_offset_s, comp_r2_s));
grid(ax2s, 'off');
sgtitle(fig3, 'LC <-> UR calibration fit -- Step 2/3: UR compensation vs load cell, sign-corrected fz');
out3_path = fullfile(OUT_DIR, 'lc_ur_calibration_step3_signed_fz_matlab.png');
print(fig3, out3_path, '-dpng', '-r150');
fprintf('Saved -> %s\n', out3_path);

% --- Figure 4: Bland-Altman diagnostic for the sign-corrected pooled
% correction (the deployable one) ---
fig4 = figure('Color', 'w', 'Position', [100 100 800 650]);
ax3 = axes(fig4);
hold(ax3, 'on');
for i = 1:numel(points)
    [color, marker, faceColor, sz, lw] = point_style(points(i));
    plot(ax3, ba_mean_pair(i), ba_diff(i), marker, 'Color', color, 'MarkerFaceColor', faceColor, ...
         'MarkerSize', sz, 'LineWidth', lw);
end
yline(ax3, ba_bias, 'Color', 'k', 'LineWidth', 2);
yline(ax3, ba_lo, 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.5);
yline(ax3, ba_hi, 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.5);
yline(ax3, 0, 'Color', [0.75 0.75 0.75]);
xlabel(ax3, 'mean(fz_{corrected}, F_{lc}) (N)');
ylabel(ax3, 'fz_{corrected} - F_{lc} (N)   [circle=posz, square=negz]');
title(ax3, sprintf('Bland-Altman -- sign-corrected pooled correction agreement with load cell\nbias=%+.3f N, LoA=[%+.3f, %+.3f] N', ...
      ba_bias, ba_lo, ba_hi));
grid(ax3, 'off');
sgtitle(fig4, 'LC <-> UR calibration fit -- Bland-Altman (sign-corrected pooled correction)');
out4_path = fullfile(OUT_DIR, 'lc_ur_calibration_step4_bland_altman_matlab.png');
print(fig4, out4_path, '-dpng', '-r150');   % 'print' works on both old MATLAB and Octave
fprintf('Saved -> %s\n', out4_path);


%% ============================= Local functions =============================
% Everything below is only usable from within this script file (MATLAB
% "local functions", supported since R2016b). They are listed in the same
% order the script calls them.

function entries = discover_entries(log_dir, instrument)
% DISCOVER_ENTRIES  Find fzcal_<instrument>_<direction>_<weight>g_<ts>.csv
% files and parse direction/weight/timestamp out of each filename. Also
% matches fzcal_<instrument>_<weight>g_<ts>.csv (no direction token,
% direction read from that file's meta.json "axis" field instead).
%
% nominal_weight_g groups repeat sessions of the same target weight by
% rounding to the nearest gram -- no hand-maintained list of expected
% weights to edit every time a new one is collected; real weights are
% already clean grams, so this groups repeats while keeping genuinely
% distinct weights apart automatically.
%
% Returns a struct array (one element per matching file) with fields:
%   instrument, direction, weight_g, nominal_weight_g, ts, version, csv_path, meta_path

    files = dir(fullfile(log_dir, sprintf('fzcal_%s_*.csv', instrument)));
    expr = ['fzcal_' instrument '_(?<direction>posz|negz)_' ...
            '(?<weight>\d+(\.\d+)?)g_(?:(?<version>v\d+)_)?(?<ts>\d{8}_\d{6})\.csv$'];
    % The 20260715 negz v2 re-collection drops the direction token from
    % the filename entirely (fzcal_futek_direct_100g_v2_..._meta.json
    % instead of fzcal_futek_direct_negz_100g_v2_...) -- direction is
    % only recoverable from that session's own meta.json ("axis" field).
    % Tried as a fallback when expr doesn't match.
    expr_no_dir = ['fzcal_' instrument '_' ...
            '(?<weight>\d+(\.\d+)?)g_(?:(?<version>v\d+)_)?(?<ts>\d{8}_\d{6})\.csv$'];

    entries_cell = {};
    for k = 1:numel(files)
        tok = regexp(files(k).name, expr, 'names');
        if ~isempty(tok)
            direction = tok.direction;
        else
            tok = regexp(files(k).name, expr_no_dir, 'names');
            if isempty(tok)
                continue
            end
            meta_path_probe = fullfile(files(k).folder, strrep(files(k).name, '.csv', '_meta.json'));
            meta_probe = jsondecode(fileread(meta_path_probe));
            direction = meta_probe.axis;
        end
        if is_excluded_session(instrument, direction, tok.ts)
            continue
        end
        weight_g = str2double(tok.weight);

        e.instrument = instrument;
        e.direction = direction;
        e.weight_g = weight_g;
        e.nominal_weight_g = round(weight_g);
        e.ts = tok.ts;
        % regexp(...,'names') only creates a field for a named token that
        % actually participated in THIS match -- un-tagged filenames (no
        % "v2_" etc.) never get a .version field at all, so tok.version
        % would error on them. isfield keeps every entry struct's field
        % set identical, which [entries_cell{:}] below also requires.
        if isfield(tok, 'version')
            e.version = tok.version;
        else
            e.version = '';
        end
        e.csv_path = fullfile(files(k).folder, files(k).name);
        e.meta_path = strrep(e.csv_path, '.csv', '_meta.json');

        entries_cell{end + 1} = e; %#ok<AGROW>
    end

    if isempty(entries_cell)
        entries = struct([]);
    else
        entries = [entries_cell{:}];
    end
end


function tf = is_excluded_session(instrument, direction, ts)
% IS_EXCLUDED_SESSION  Manual exclusion, confirmed by whoever ran the
% collection: these two ur_only negz files (labeled 201g and 202g) are
% one-off mislabeled attempts that don't represent a real, distinct test
% point -- ignore them entirely. The plain 20g file
% (fzcal_ur_only_negz_20g_..._180618.csv) is used for the 20g point
% instead.

    tf = strcmp(instrument, 'ur_only') && strcmp(direction, 'negz') && ...
         (strcmp(ts, '20260706_181552') || strcmp(ts, '20260706_181726'));
end


function v = parse_version(version_str)
% PARSE_VERSION  Un-tagged filenames (no "_v2_" etc.) are implicitly
% version 1.

    if isempty(version_str)
        v = 1;
    else
        v = str2double(version_str(2:end));  % strip leading 'v'
    end
end


function kept = keep_latest_version(entries)
% KEEP_LATEST_VERSION  Each full re-collection (v1 implied, then v2, v3,
% ...) fully supersedes the previous one, PER INSTRUMENT: a leftover
% session from an older batch with no counterpart in the newest one
% (e.g. a v1-only weight, no vN match) has its own baseline reading,
% which drifts ~1 N session-to-session -- see the v2 fix that motivated
% this (git history) -- so mixing an old batch's leftover into the
% newest batch corrupts the fit instead of adding data. Keeps only the
% highest version number present for each instrument; hardcodes no
% specific version number, so the next re-collection (v3, v4, ...) is
% picked up with no code change.

    if isempty(entries)
        kept = entries;
        return
    end
    instruments = unique({entries.instrument});
    kept_cell = cell(1, numel(instruments));
    for i = 1:numel(instruments)
        group = entries(strcmp({entries.instrument}, instruments{i}));
        versions = arrayfun(@(e) parse_version(e.version), group);
        kept_cell{i} = group(versions == max(versions));
    end
    kept = [kept_cell{:}];
end


function print_dataset_manifest(entries, label)
% PRINT_DATASET_MANIFEST  Easy way to see exactly which files feed a
% given instrument's fit -- always printed (unlike the [dedupe] lines,
% which only fire when a group had more than one candidate), so a
% re-collection's effect on what's in use is visible at a glance.

    fprintf('\n[datasets] %s: %d file(s) in use\n', label, numel(entries));
    if isempty(entries)
        return
    end
    keys = cell(1, numel(entries));
    for i = 1:numel(entries)
        keys{i} = sprintf('%s_%08.2f', entries(i).direction, entries(i).nominal_weight_g);
    end
    [~, order] = sort(keys);
    entries = entries(order);
    for i = 1:numel(entries)
        e = entries(i);
        if isempty(e.version)
            version_label = 'v1 (untagged)';
        else
            version_label = e.version;
        end
        [~, fname, ext] = fileparts(e.csv_path);
        fprintf('  %4s %6.0fg  %-14s %s%s\n', e.direction, e.nominal_weight_g, version_label, fname, ext);
    end
end


function kept = dedupe_latest(entries)
% DEDUPE_LATEST  Group entries by (instrument, direction, nominal weight)
% and keep only the chronologically-last file in each group.

    keys = cell(1, numel(entries));
    for i = 1:numel(entries)
        keys{i} = sprintf('%s|%s|%g', entries(i).instrument, entries(i).direction, ...
                           entries(i).nominal_weight_g);
    end
    unique_keys = unique(keys);

    kept_cell = cell(1, numel(unique_keys));
    for k = 1:numel(unique_keys)
        idx = find(strcmp(keys, unique_keys{k}));
        group = entries(idx);
        [~, order] = sort({group.ts});
        group = group(order);

        if numel(group) > 1
            dropped_labels = cell(1, numel(group) - 1);
            for j = 1:numel(group) - 1
                dropped_labels{j} = sprintf('%gg@%s', group(j).weight_g, group(j).ts);
            end
            fprintf('[dedupe] %-12s %-4s %5.0fg: keeping %gg@%s (dropping %s)\n', ...
                    group(end).instrument, group(end).direction, group(end).nominal_weight_g, ...
                    group(end).weight_g, group(end).ts, strjoin(dropped_labels, ', '));
        end
        kept_cell{k} = group(end);
    end

    if isempty(kept_cell)
        kept = struct([]);
    else
        kept = [kept_cell{:}];
    end
end


function s = load_session(entry, G)
% LOAD_SESSION  Read one calibration CSV + its meta json, and compute the
% baseline and loaded means for fz (both instruments) and ai0
% (futek_direct sessions only -- ur_only has no load cell installed).
%
% Every log has exactly one no-load block (loaded==0) followed by one
% loaded block (loaded==1).

    meta = jsondecode(fileread(entry.meta_path));
    T = readtable(entry.csv_path);

    is_loaded = T.loaded == 1;
    is_base = T.loaded == 0;

    s = entry;
    s.tilt_deg = meta.tilt_from_vertical_deg;
    s.fz_base_mean = mean(T.fz(is_base));
    s.fz_load_mean = mean(T.fz(is_loaded));
    s.dfz_mean = s.fz_load_mean - s.fz_base_mean;   % kept for reference/diagnostics only
    s.dfz_std = std(T.fz(is_loaded) - s.fz_base_mean);

    if strcmp(entry.instrument, 'futek_direct')
        s.ai0_base_mean = mean(T.ai0(is_base));
        s.ai0_load_mean = mean(T.ai0(is_loaded));
        s.dv_mean = s.ai0_load_mean - s.ai0_base_mean;
        s.dv_std = std(T.ai0(is_loaded) - s.ai0_base_mean);
    end

    tilt_rad = deg2rad(meta.tilt_from_vertical_deg);
    if strcmp(entry.instrument, 'ur_only')
        hardware_g = extra_hardware_ur_only(entry.direction);
    else
        hardware_g = extra_hardware_futek_direct(entry.direction);
    end
    total_g = entry.weight_g + hardware_g;

    s.F_true_base = (hardware_g / 1000) * G * cos(tilt_rad);
    s.F_true = (total_g / 1000) * G * cos(tilt_rad);
    % F_signed/F_signed_base orient the ground truth by measurement
    % direction (push=posz vs pull=negz), same ai0_sign convention used to
    % sign fz -- for BOTH instruments, since direction is a property of
    % the experiment, not of which sensor is reading it. Needed so a
    % sign-corrected fz can be pooled against a ground truth that's ALSO
    % on the same sign convention (see plot_ur_only_vs_load.m).
    s.F_signed = ai0_sign(entry.direction) * s.F_true;
    s.F_signed_base = ai0_sign(entry.direction) * s.F_true_base;
    if strcmp(entry.instrument, 'futek_direct')
        % Separate ground truth for the UR sensor itself in this SAME
        % rig: the UR holds up the load cell's own body too, not just
        % what the load cell feels (see extra_hardware_futek_direct_ur).
        hardware_g_ur = extra_hardware_futek_direct_ur(entry.direction);
        total_g_ur = entry.weight_g + hardware_g_ur;
        s.F_true_ur_base = (hardware_g_ur / 1000) * G * cos(tilt_rad);
        s.F_true_ur = (total_g_ur / 1000) * G * cos(tilt_rad);
        s.F_signed_ur_base = ai0_sign(entry.direction) * s.F_true_ur_base;
        s.F_signed_ur = ai0_sign(entry.direction) * s.F_true_ur;
    end
end


function points = expand_phases(sessions)
% EXPAND_PHASES  Turn futek_direct sessions into a flat struct array of
% per-phase points, 2 per session (baseline then loaded) -- baseline is a
% known, non-zero load here (see script header), not a zero reference.

    points_cell = cell(1, 2 * numel(sessions));
    for i = 1:numel(sessions)
        s = sessions(i);

        hardware_g = extra_hardware_futek_direct(s.direction);

        b.weight_g = s.weight_g; b.direction = s.direction; b.phase = 'baseline';
        b.ai0 = s.ai0_base_mean; b.fz = s.fz_base_mean; b.F_signed = s.F_signed_base;
        b.fz_signed = ai0_sign(s.direction) * abs(s.fz_base_mean);
        b.compensated_g = hardware_g;

        l.weight_g = s.weight_g; l.direction = s.direction; l.phase = 'loaded';
        l.ai0 = s.ai0_load_mean; l.fz = s.fz_load_mean; l.F_signed = s.F_signed;
        l.fz_signed = ai0_sign(s.direction) * abs(s.fz_load_mean);
        l.compensated_g = s.weight_g + hardware_g;

        points_cell{2 * i - 1} = b;
        points_cell{2 * i} = l;
    end
    points = [points_cell{:}];
end


function sorted = sort_sessions(sessions)
% SORT_SESSIONS  Order a session struct array by direction then weight_g,
% purely so printed tables and plots come out in a readable order.

    keys = cell(1, numel(sessions));
    for i = 1:numel(sessions)
        keys{i} = sprintf('%s_%08.2f', sessions(i).direction, sessions(i).weight_g);
    end
    [~, order] = sort(keys);
    sorted = sessions(order);
end


function [m, c, r2, rmse] = linfit(x, y)
% LINFIT  Ordinary least-squares line  y = m*x + c , plus R^2 and RMSE.

    x = x(:);
    y = y(:);
    p = polyfit(x, y, 1);
    m = p(1);
    c = p(2);

    y_pred = m * x + c;
    ss_res = sum((y - y_pred) .^ 2);
    ss_tot = sum((y - mean(y)) .^ 2);
    r2 = 1 - ss_res / ss_tot;
    rmse = sqrt(mean((y - y_pred) .^ 2));
end


function sgn = ai0_sign(direction)
% AI0_SIGN  Load-cell voltage sign convention: +z (posz) pushes the
% bridge voltage down, -z (negz) pulls it up. This is a hardware fact
% (checked against raw data), not an assumption.

    if strcmp(direction, 'posz')
        sgn = -1;
    else
        sgn = 1;
    end
end


function extra_g = extra_hardware_ur_only(direction)
% EXTRA_HARDWARE_UR_ONLY  Hardware mass (g) felt by the UR sensor in the
% ur_only (no load cell) rig, on top of the nominal test weight: 43 g
% (posz) / 37 g (negz).

    if strcmp(direction, 'posz')
        extra_g = 43;
    else
        extra_g = 37;
    end
end


function extra_g = extra_hardware_futek_direct(direction)
% EXTRA_HARDWARE_FUTEK_DIRECT  Hardware mass (g) felt by the load cell's
% OWN reading in the futek_direct rig, on top of the nominal test weight:
% only what's mounted ABOVE the load cell in the load path -- the holder
% (7 g, posz, same physical holder as ur_only) or the hook (4 g, negz --
% a DIFFERENT, heavier hook than the 1 g one used in ur_only; confirmed
% by whoever ran the collection, not an inconsistency). Used ONLY for the
% ai0<->force (Step 1) fit's ground truth.

    if strcmp(direction, 'posz')
        extra_g = 7;
    else
        extra_g = 4;
    end
end


function extra_g = extra_hardware_futek_direct_ur(direction)
% EXTRA_HARDWARE_FUTEK_DIRECT_UR  Hardware mass (g) felt by the UR sensor
% itself in the SAME futek_direct rig: the UR holds up everything below
% it in the load path -- the 3D-printed coupler (15 g, common to every
% experiment) + 4 attachment screws (21 g) + the load cell's own body
% (7 g) = 43 g, common to both directions, plus the holder (7 g, posz) or
% the hook (4 g, negz) above the load cell. Used for any ground truth the
% UR's fz is compared against directly (bypassing the load cell) -- NOT
% for the ai0 fit, which uses extra_hardware_futek_direct instead.

    if strcmp(direction, 'posz')
        extra_g = 50;
    else
        extra_g = 47;
    end
end


function [color, marker, faceColor, sz, lw] = point_style(p)
% POINT_STYLE  Marker styling for one expand_phases() point: color by
% weight, shape by direction (circle=posz, square=negz), filled for the
% loaded phase and open (unfilled) for the baseline phase.

    color = weight_color(p.weight_g);
    if strcmp(p.direction, 'posz')
        marker = 'o';
    else
        marker = 's';
    end
    if strcmp(p.phase, 'loaded')
        faceColor = color;
        sz = 9;
        lw = 1.0;
    else
        faceColor = 'none';
        sz = 7;
        lw = 1.5;
    end
end


function c = weight_color(weight_g)
% WEIGHT_COLOR  Fixed color per nominal calibration weight, so the same
% weight always plots in the same color across figures.

    switch round(weight_g)
        case 5,   c = [0.12 0.47 0.71];
        case 10,  c = [1.00 0.50 0.05];
        case 20,  c = [0.17 0.63 0.17];
        case 50,  c = [0.84 0.15 0.16];
        case 100, c = [0.58 0.40 0.74];
        case 200, c = [0.55 0.34 0.29];
        otherwise, c = [0.2 0.2 0.2];
    end
end
