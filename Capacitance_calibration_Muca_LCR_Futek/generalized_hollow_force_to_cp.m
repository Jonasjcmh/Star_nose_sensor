% =========================================================================
% generalized_hollow_force_to_cp.m
%
% Alternative to the per-surface STEP B fits in
% calibrate_capacitance_muca_lcr_futek.m: instead of needing a separate
% Force<->Cp_pF relationship per surface (and being stuck with "n/a" for
% 14/19 hollow points that have no direct LCR coverage), this pools
% HOLLOW's own LCR sweep -- the CLEANEST signal available (per-point R^2
% 0.96-0.995 in per_point_surface_relationships.md, vs. much noisier
% per-point fits on flat/solid) -- into ONE Force -> Cp_pF model, and
% checks whether that single relationship generalizes to flat and solid
% too, BEFORE applying it universally.
%
% This is a real assumption (that Cp-vs-Force is roughly surface-
% independent), not a given -- so step 2 below is a genuine
% cross-validation, not just a sanity check: the hollow-derived model is
% applied to flat's and solid's OWN (Force, Cp_pF) LCR points and scored
% against their ACTUAL measured Cp_pF, exactly like testing a model on
% held-out data it never saw.
%
% Run AFTER calibrate_capacitance_muca_lcr_futek.m (reuses its shared
% helper files).
% =========================================================================

clear; close all; clc;

function [dc_pool, F_pool] = normalize_by_own_c0(rows)
% Octave requires local script-functions defined before first use.
% rows: [point, depth_mm, force_N, Cp_pF]. Returns pooled (Force,
% delta-C/C0) across all points in rows, C0 = each point's own min Cp.
    pts = unique(rows(:, 1));
    dc_pool = [];
    F_pool = [];
    for i = 1:numel(pts)
        sub = rows(rows(:, 1) == pts(i), :);
        c0 = min(sub(:, 4));
        if c0 == 0, continue; end
        dc_pool = [dc_pool; (sub(:, 4) - c0) / c0]; %#ok<AGROW>
        F_pool  = [F_pool; sub(:, 3)]; %#ok<AGROW>
    end
end

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
fprintf('  GENERALIZED Force -> Cp_pF MODEL (derived from HOLLOW, applied to ALL surfaces)\n');
fprintf('%s\n', repmat('=', 1, 74));

%% ---- STEP 1: fit the general model on hollow's own LCR sweep -----------
fprintf('\n[STEP 1] Fit Force -> Cp_pF pooling ALL of hollow''s LCR sweep\n');
fprintf('%s\n', repmat('-', 1, 74));
lcr_rows_by_surface = cell(1, 3);
for s = 1:3
    lcr_d = load_lcr_files(LCR_FILES{s});
    lcr_rows_by_surface{s} = lcr_stats_by_point_depth(lcr_d);   % [point,depth,force,Cp_pF]
end
hollow_rows = lcr_rows_by_surface{3};
fprintf('  hollow LCR sweep: %d (point,depth) rows, %d distinct point(s)\n', ...
    size(hollow_rows, 1), numel(unique(hollow_rows(:, 1))));

F_h  = hollow_rows(:, 3);
Cp_h = hollow_rows(:, 4);
general_fit = polyfit(F_h, Cp_h, 1);
yfit_h = polyval(general_fit, F_h);
r2_h = 1 - sum((Cp_h - yfit_h).^2) / sum((Cp_h - mean(Cp_h)).^2);
fprintf('  GENERAL MODEL (from hollow, pooled across all 19 points): Cp_pF = %.5f * Force_N + %.5f\n', ...
    general_fit(1), general_fit(2));
fprintf('  Fit on hollow''s own data: R2=%.3f, n=%d\n', r2_h, numel(F_h));

%% ---- STEP 2: cross-validate on flat and solid's OWN LCR data ------------
fprintf('\n[STEP 2] Cross-validation: does hollow''s Force->Cp_pF model predict flat/solid?\n');
fprintf('%s\n', repmat('-', 1, 74));
fprintf(['  Applying the HOLLOW-derived model to flat''s and solid''s own (Force,Cp_pF)\n' ...
    '  LCR points -- these are real measurements the model has never seen, so this is\n' ...
    '  a genuine held-out check, not circular.\n\n']);
xval_r2 = nan(1, 3);
for s = 1:3
    rows = lcr_rows_by_surface{s};
    F = rows(:, 3);
    Cp_actual = rows(:, 4);
    Cp_pred = polyval(general_fit, F);
    resid = Cp_actual - Cp_pred;
    ss_res = sum(resid .^ 2);
    ss_tot = sum((Cp_actual - mean(Cp_actual)) .^ 2);
    r2 = 1 - ss_res / max(ss_tot, eps);
    rmse = sqrt(mean(resid .^ 2));
    xval_r2(s) = r2;
    tag = '';
    if s == 3, tag = '  (in-sample -- this is where the model came from)'; end
    fprintf('  %-7s: R2=%.3f  RMSE=%.4f pF  n=%d%s\n', SURFACE_NAMES{s}, r2, rmse, numel(F), tag);
end
fprintf(['\n  ASSESSMENT: R2 near hollow''s own (%.3f) on flat/solid would support generalizing;\n' ...
    '  a much lower or negative R2 on flat/solid means Cp-vs-Force is surface-DEPENDENT and\n' ...
    '  this generalization trades away accuracy for full 19-point coverage. See numbers above\n' ...
    '  before trusting the applied results below.\n'], r2_h);

%% ---- STEP 1B: try the SAME idea but on delta-C/C0 (baseline-normalized) --
% STEP 1's pooled fit on RAW Cp_pF got R2~0 even in-sample on hollow --
% each point has such a different absolute capacitance baseline that
% pooling raw pF across points erases the real per-point relationship
% (same root cause the README already documents for the V-based pooled
% fits). The standard fix for exactly this is to normalize each point's
% own sweep by its own baseline BEFORE pooling: delta_C/C0 = (Cp - C0)/C0,
% where C0 = that point's minimum observed Cp (same convention as
% lcr_delta_c_over_c0.m). This removes the baseline mismatch; if points
% still don't share a common Force-relationship after that, the surfaces
% are genuinely different, not just offset.
fprintf('\n[STEP 1B] Same idea, but pooling delta-C/C0 (baseline-normalized) instead of raw Cp_pF\n');
fprintf('%s\n', repmat('-', 1, 74));

[dc_hollow, F_dc_hollow] = normalize_by_own_c0(hollow_rows);
general_fit_norm = polyfit(F_dc_hollow, dc_hollow, 1);
yfit_n = polyval(general_fit_norm, F_dc_hollow);
r2_n = 1 - sum((dc_hollow - yfit_n).^2) / sum((dc_hollow - mean(dc_hollow)).^2);
fprintf('  GENERAL MODEL (normalized, from hollow): delta_C/C0 = %.5f * Force_N + %.5f\n', ...
    general_fit_norm(1), general_fit_norm(2));
fprintf('  Fit on hollow''s own data: R2=%.3f, n=%d\n\n', r2_n, numel(F_dc_hollow));

xval_r2_norm = nan(1, 3);
for s = 1:3
    [dc_s, F_dc_s] = normalize_by_own_c0(lcr_rows_by_surface{s});
    pred = polyval(general_fit_norm, F_dc_s);
    resid = dc_s - pred;
    r2 = 1 - sum(resid.^2) / max(sum((dc_s - mean(dc_s)).^2), eps);
    rmse = sqrt(mean(resid.^2));
    xval_r2_norm(s) = r2;
    tag = '';
    if s == 3, tag = '  (in-sample)'; end
    fprintf('  %-7s: R2=%.3f  RMSE=%.4f (unitless delta-C/C0)  n=%d%s\n', ...
        SURFACE_NAMES{s}, r2, rmse, numel(F_dc_s), tag);
end
fprintf(['\n  ASSESSMENT: compare these R2 against STEP 2''s raw-pF numbers above. If\n' ...
    '  normalized R2 is meaningfully higher across surfaces, delta-C/C0 (not raw pF) is\n' ...
    '  the quantity that actually generalizes -- use it instead for a universal model.\n']);

%% ---- STEP 3: apply the general model to the FULL mucaboard_data_raw ----
fprintf('\n[STEP 3] Applying the general Force->Cp_pF model to every mucaboard_data_raw reading\n');
fprintf('%s\n', repmat('-', 1, 74));

out_fid = fopen(fullfile(RESULTS_DIR, 'mucaboard_data_raw_in_pF_generalized_hollow_model.csv'), 'w');
fprintf(out_fid, 'surface,true_point,round_idx,phase,raw_own,force_N,Cp_pF_estimated_general\n');

Cp_hexmap = nan(19, 3);
a = general_fit(1);
b = general_fit(2);

for s = 1:3
    d = read_muca_raw_csv(MUCA_RAW_FILES{s}, RAW_VALID_MAX);
    for true_pt = 1:19
        code_id = TRUE_TO_CODE(true_pt);
        mask = (d.ur5_point == code_id) & (d.round_idx >= 0) & (d.round_idx <= N_ITERATIONS - 1) & ...
               (strcmp(d.phase, 'hold') | strcmp(d.phase, 'press') | strcmp(d.phase, 'retract'));
        idx = find(mask);
        if isempty(idx)
            continue;
        end
        raw_vals   = d.raw_cells(idx, code_id);
        force_vals = d.load_cell_N(idx);
        cp_vals    = a * force_vals + b;
        for i = 1:numel(idx)
            fprintf(out_fid, '%s,%d,%d,%s,%.4f,%.4f,%.5f\n', SURFACE_NAMES{s}, true_pt, ...
                d.round_idx(idx(i)), d.phase{idx(i)}, raw_vals(i), force_vals(i), cp_vals(i));
        end

        hold_mask = mask & strcmp(d.phase, 'hold');
        if any(hold_mask)
            force_h = mean(d.load_cell_N(hold_mask), 'omitnan');
            Cp_hexmap(true_pt, s) = a * force_h + b;
        end
    end
    fprintf('  %-7s: wrote per-sample Cp_pF for all 19 points using the general Force->Cp_pF model\n', ...
        SURFACE_NAMES{s});
end
fclose(out_fid);
fprintf('\n  Saved: %s\n', fullfile(RESULTS_DIR, 'mucaboard_data_raw_in_pF_generalized_hollow_model.csv'));
fprintf('  (Note: this model uses FORCE directly, not the muca V reading -- unlike\n');
fprintf('  calibrate_capacitance_muca_lcr_futek.m''s STEP B output, which is V-based.)\n');

%% ---- hex-map figure -------------------------------------------------------
[point_ids, point_xy, ~] = muca_layout();
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
    ylabel(cb, 'Estimated Cp (pF), general hollow-derived Force model');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
    end
    fig_suptitle(fig, sprintf('mucaboard\\_data\\_raw -- Cp\\_pF = %.4f*Force + %.4f (hollow-derived, all surfaces)', a, b));
    save_fig(fig, fullfile(RESULTS_DIR, 'mucaboard_generalized_hollow_model_hexmap'));
    fprintf('  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR, 'mucaboard_generalized_hollow_model_hexmap'));
end

%% ---- cross-validation scatter figure ------------------------------------
figXV = figure('Position', [100 100 700 600]);
hold on; box on; grid on;
colors = [0.9490 0.7804 0.3608; 0.8784 0.4941 0.2353; 0.4784 0.3765 0.2510];
for s = 1:3
    rows = lcr_rows_by_surface{s};
    scatter(rows(:, 3), rows(:, 4), 35, colors(s, :), 'filled', 'DisplayName', SURFACE_NAMES{s});
end
Fline = linspace(min(cellfun(@(r) min(r(:,3)), lcr_rows_by_surface)), ...
                  max(cellfun(@(r) max(r(:,3)), lcr_rows_by_surface)), 50);
plot(Fline, polyval(general_fit, Fline), 'k-', 'LineWidth', 2, ...
    'DisplayName', sprintf('general (hollow-derived): Cp=%.4f*F+%.4f', a, b));
xlabel('Force (N)');
ylabel('Cp\_pF');
title('Hollow-derived Force->Cp\_pF model vs. all 3 surfaces'' own LCR data');
legend('Location', 'best', 'Interpreter', 'none');
save_fig(figXV, fullfile(RESULTS_DIR, 'generalized_hollow_model_crossvalidation'));
fprintf('  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR, 'generalized_hollow_model_crossvalidation'));

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('  DONE.\n');
fprintf('%s\n', repmat('=', 1, 74));
