% =========================================================================
% generalized_hollow_muca_v_to_cp.m
%
% Tests a specific, physically-motivated hypothesis: generalized_hollow_
% force_to_cp.m already showed Force<->Cp_pF is surface-DEPENDENT (fails
% cross-validation even after baseline-normalizing) -- that's a material-
% mechanics relationship (how much a surface deforms/couples under a given
% force), which has every reason to differ between flat/solid/hollow.
%
% But muca_V <-> Cp_pF is a DIFFERENT relationship: the FT5x16 chip's own
% raw-count response to a given amount of capacitance at one of its 19
% RX/TX electrode pairs. That's arguably a fixed HARDWARE/FIRMWARE
% property -- same chip, same 19 pads, same ADC + SENSITIVITY/GAMMA
% normalization -- regardless of what surface material is sitting on top
% causing the capacitance change. If that's right, hollow's 5-point
% muca_V<->Cp_pF relationship (the CLEANEST paired data available, since
% hollow's own per-point Force<->Cp fits were R^2 0.96-0.995) should
% generalize to flat/solid better than the Force-based model did.
%
% This is tested exactly like generalized_hollow_force_to_cp.m: fit on
% hollow's matched (V, Cp_pF) pairs, cross-validate against flat's and
% solid's OWN matched (V, Cp_pF) pairs (force-interpolated from
% Capacitance_measurement, same construction as STEP B / per_point_
% surface_relationships.m -- NOT circular, those are independently
% matched per surface).
%
% Run AFTER calibrate_capacitance_muca_lcr_futek.m (reuses its helpers).
% =========================================================================

clear; close all; clc;

HERE          = fileparts(mfilename('fullpath'));
REPO_ROOT     = fullfile(HERE, '..');
MUCA_TOOLKIT  = fullfile(REPO_ROOT, 'mucaboard_data', 'matlab');
MUCA_RAW_LOGS = fullfile(REPO_ROOT, 'mucaboard_data_raw', 'logs');
CAPMEAS_LOGS  = fullfile(REPO_ROOT, 'Capacitance_measurement', 'logs');
RESULTS_DIR   = fullfile(HERE, 'results');
if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end
addpath(MUCA_TOOLKIT);

TRUE_TO_CODE  = [8,4,1,13,9,5,2,17,14,10,6,3,18,15,11,7,19,16,12];
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
% mucaboard_data_raw's actual press depth -- 10mm -- is covered directly).
% Supersedes the old partial coverage (hollow was only 5/19 points).
LCR_FILES = { ...
    { fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260828_193051_flat_sensor.csv') }, ...
    { fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260828_162718_solid_sensor.csv') }, ...
    { fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260825_202746_hollow_sensor.csv') } ...
};

fprintf('%s\n', repmat('=', 1, 74));
fprintf('  TESTING: does hollow''s muca_V <-> Cp_pF relationship generalize to flat/solid?\n');
fprintf('  (as opposed to Force <-> Cp_pF, already shown surface-dependent)\n');
fprintf('%s\n', repmat('=', 1, 74));

%% ---- build matched (V, Cp_pF) pairs per surface, exactly like STEP B ---
matched_V  = cell(1, 3);
matched_Cp = cell(1, 3);
matched_pt = cell(1, 3);

for s = 1:3
    d = read_muca_raw_csv(MUCA_RAW_FILES{s}, RAW_VALID_MAX);
    rows = muca_raw_own_reading_by_point(d, TRUE_TO_CODE, N_ITERATIONS, SENSITIVITY, GAMMA);
    lcr_d = load_lcr_files(LCR_FILES{s});
    lcr_rows = lcr_stats_by_point_depth(lcr_d);

    v_list = []; cp_list = []; pt_list = [];
    for true_pt = 1:19
        if isnan(rows(true_pt, 1)), continue; end
        code_id = TRUE_TO_CODE(true_pt);
        sub = lcr_rows(lcr_rows(:, 1) == code_id, [3 4]);
        if isempty(sub), continue; end
        [cp_est, ~] = interp_force_to_cp(sub, rows(true_pt, 2));
        if isnan(cp_est), continue; end
        v_list(end+1)  = rows(true_pt, 4); %#ok<AGROW>
        cp_list(end+1) = cp_est; %#ok<AGROW>
        pt_list(end+1) = true_pt; %#ok<AGROW>
    end
    matched_V{s}  = v_list;
    matched_Cp{s} = cp_list;
    matched_pt{s} = pt_list;
    fprintf('  %-7s: %d matched (V, Cp_pF) points\n', SURFACE_NAMES{s}, numel(v_list));
end

%% ---- STEP 1: fit on hollow's own matched pairs --------------------------
fprintf('\n[STEP 1] Fit muca_V -> Cp_pF on hollow''s own matched pairs\n');
fprintf('%s\n', repmat('-', 1, 74));
Vh = matched_V{3};
Cph = matched_Cp{3};
hollow_fit = polyfit(Vh, Cph, 1);
yfit_h = polyval(hollow_fit, Vh);
r2_h = 1 - sum((Cph - yfit_h).^2) / max(sum((Cph - mean(Cph)).^2), eps);
fprintf('  GENERAL MODEL (from hollow, its %d matched points): Cp_pF = %.4f * V_own + %.4f\n', numel(Vh), ...
    hollow_fit(1), hollow_fit(2));
fprintf('  Fit on hollow''s own data: R2=%.3f, n=%d  (same numbers as STEP B in\n', r2_h, numel(Vh));
fprintf('  calibrate_capacitance_muca_lcr_futek.m -- this IS that fit, just isolated here)\n');

%% ---- STEP 2: cross-validate on flat/solid's OWN matched pairs -----------
fprintf('\n[STEP 2] Cross-validation on flat''s and solid''s OWN matched (V, Cp_pF) pairs\n');
fprintf('%s\n', repmat('-', 1, 74));
fprintf(['  For comparison, each surface''s OWN pooled fit (from calibrate_capacitance_\n' ...
    '  muca_lcr_futek.m STEP B) is shown alongside -- that''s the best a same-surface\n' ...
    '  model can do with this data; hollow''s cross-validated R2 should be compared\n' ...
    '  against THAT ceiling, not against 0.\n\n']);
fprintf('  %-7s %10s %10s %14s %10s\n', 'surface', 'n', 'R2(hollow', 'model)', 'R2(own fit)');
xval_r2 = nan(1, 3);
own_r2  = nan(1, 3);
for s = 1:3
    V = matched_V{s};
    Cp = matched_Cp{s};
    pred = polyval(hollow_fit, V);
    resid = Cp - pred;
    r2 = 1 - sum(resid.^2) / max(sum((Cp - mean(Cp)).^2), eps);
    xval_r2(s) = r2;

    if numel(V) >= 2
        ownfit = polyfit(V, Cp, 1);
        ownpred = polyval(ownfit, V);
        own_r2(s) = 1 - sum((Cp - ownpred).^2) / max(sum((Cp - mean(Cp)).^2), eps);
    end
    tag = '';
    if s == 3, tag = ' (in-sample)'; end
    fprintf('  %-7s %10d %10.3f %14s %10.3f%s\n', SURFACE_NAMES{s}, numel(V), r2, '', own_r2(s), tag);
end

fprintf(['\n  ASSESSMENT: if hollow-model R2 on flat/solid is close to (or better than) their\n' ...
    '  OWN same-surface pooled fit R2, hollow''s muca_V<->Cp_pF relationship is a\n' ...
    '  reasonable stand-in / improvement. If it''s much worse (including negative), the\n' ...
    '  hardware-transfer-function hypothesis does not hold up either -- V<->Cp_pF is\n' ...
    '  ALSO surface-dependent (plausible: different surfaces still change the ELECTRIC\n' ...
    '  FIELD geometry at the electrode differently, not just the mechanical coupling).\n']);

%% ---- STEP 3: RMSE in real units too (R2 alone can be misleading with n=5) --
fprintf('\n[STEP 3] RMSE in pF (R2 alone can be misleading -- check magnitude too)\n');
fprintf('%s\n', repmat('-', 1, 74));
for s = 1:3
    V = matched_V{s};
    Cp = matched_Cp{s};
    pred = polyval(hollow_fit, V);
    rmse = sqrt(mean((Cp - pred).^2));
    cp_range = max(Cp) - min(Cp);
    fprintf('  %-7s: RMSE=%.4f pF  (that surface''s own Cp range spans %.4f pF)\n', ...
        SURFACE_NAMES{s}, rmse, cp_range);
end

%% ---- scatter figure -------------------------------------------------------
fig = figure('Position', [100 100 700 600]);
hold on; box on; grid on;
colors = [0.9490 0.7804 0.3608; 0.8784 0.4941 0.2353; 0.4784 0.3765 0.2510];
for s = 1:3
    scatter(matched_V{s}, matched_Cp{s}, 45, colors(s, :), 'filled', 'DisplayName', SURFACE_NAMES{s});
end
Vline = linspace(0, 1, 50);
plot(Vline, polyval(hollow_fit, Vline), 'k-', 'LineWidth', 2, ...
    'DisplayName', sprintf('hollow-derived: Cp=%.3f*V+%.3f', hollow_fit(1), hollow_fit(2)));
xlabel('muca V_{own} (normalized [0,1] reading)');
ylabel('Cp\_pF (force-matched)');
title('Hollow-derived muca\_V<->Cp\_pF model vs. all 3 surfaces'' matched points');
legend('Location', 'best', 'Interpreter', 'none');
save_fig(fig, fullfile(RESULTS_DIR, 'generalized_hollow_muca_v_crossvalidation'));
fprintf('\n  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR, 'generalized_hollow_muca_v_crossvalidation'));

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('  DONE.\n');
fprintf('%s\n', repmat('=', 1, 74));
