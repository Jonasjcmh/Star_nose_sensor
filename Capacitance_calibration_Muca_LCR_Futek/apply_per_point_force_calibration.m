% =========================================================================
% apply_per_point_force_calibration.m
%
% Direct answer to "isn't Force->Cp_pF the best option, per surface?":
% YES, Force is the right variable (both generalization scripts already
% showed it's the strongest available signal), but POOLING all 19 points'
% Force<->Cp_pF into ONE line per surface repeats the exact baseline-
% mismatch mistake that broke the cross-surface generalization attempts
% (different pads have very different absolute Cp -- pooling erases the
% real relationship; see generalized_hollow_force_to_cp.m STEP 1).
%
% The fix: don't pool. per_point_surface_relationships.m already computed
% a REAL per-point Force<->Cp_pF regression for every point that has >=2
% LCR depths (R^2 mostly 0.5-0.99 -- see results/per_point_surface_
% relationships.csv). mucaboard_data_raw logs the Futek force
% CONTINUOUSLY, for every single sample, and we know exactly which point
% every sample belongs to. So instead of one pooled-per-surface equation
% applied to a single hold-mean, this applies EACH POINT'S OWN regression
% to EVERY SAMPLE of that point, using that sample's own live force
% reading -- the most direct, best-supported calibration available from
% this data.
%
% Points with fewer than 2 LCR depths (n/a in per_point_surface_
% relationships.csv -- 14/19 on hollow, per the missing-LCR-coverage
% discussion) get NO estimate here (Cp_pF left blank) rather than a
% guessed/generalized one, since both generalization attempts already
% failed cross-validation.
%
% Run AFTER per_point_surface_relationships.m (reads its output CSV).
% =========================================================================

clear; close all; clc;

HERE          = fileparts(mfilename('fullpath'));
REPO_ROOT     = fullfile(HERE, '..');
MUCA_TOOLKIT  = fullfile(REPO_ROOT, 'mucaboard_data', 'matlab');
MUCA_RAW_LOGS = fullfile(REPO_ROOT, 'mucaboard_data_raw', 'logs');
RESULTS_DIR   = fullfile(HERE, 'results');
addpath(MUCA_TOOLKIT);

TRUE_TO_CODE  = [8,4,1,13,9,5,2,17,14,10,6,3,18,15,11,7,19,16,12];
RAW_VALID_MAX = 1000;
N_ITERATIONS  = 10;
SURFACE_NAMES = {'flat', 'solid', 'hollow'};

MUCA_RAW_FILES = { ...
    fullfile(MUCA_RAW_LOGS, 'flat_sensor_19_points_10_iterations_session_20260826_155255.csv'), ...
    fullfile(MUCA_RAW_LOGS, 'solid_sensor_19_points_10_iterations_session_20260826_165651.csv'), ...
    fullfile(MUCA_RAW_LOGS, 'hollow_sensor_iterations_all_session_20260826_190632.csv') ...
};

fprintf('%s\n', repmat('=', 1, 74));
fprintf('  APPLYING PER-POINT (not pooled) Force -> Cp_pF regressions\n');
fprintf('%s\n', repmat('=', 1, 74));

%% ---- load the per-point fits computed by per_point_surface_relationships.m
rel_csv = fullfile(RESULTS_DIR, 'per_point_surface_relationships.csv');
if ~exist(rel_csv, 'file')
    error('apply_per_point_force_calibration:missing', ...
        'Run per_point_surface_relationships.m first -- %s not found.', rel_csv);
end
fid = fopen(rel_csv, 'r');
header_line = fgetl(fid);
header = strsplit(header_line, ',', 'CollapseDelimiters', false);
fmt = repmat('%s', 1, numel(header));
Crel = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
fclose(fid);
colr = @(name) Crel{find(strcmp(header, name), 1)};
rel_surface = colr('surface');
rel_point   = str2double(colr('true_point'));
rel_slope   = str2double(colr('force_cp_slope'));
rel_int     = str2double(colr('force_cp_intercept'));
rel_r2      = str2double(colr('force_cp_r2'));
rel_n       = str2double(colr('force_cp_n'));

%% ---- apply, sample by sample, to the FULL mucaboard_data_raw dataset ----
out_fid = fopen(fullfile(RESULTS_DIR, 'mucaboard_data_raw_in_pF_per_point_force.csv'), 'w');
fprintf(out_fid, ['surface,true_point,round_idx,phase,raw_own,force_N,' ...
    'Cp_pF_estimated,fit_r2,fit_n\n']);

n_covered   = 0;
n_uncovered = 0;
Cp_hexmap = nan(19, 3);
r2_hexmap = nan(19, 3);

for s = 1:3
    d = read_muca_raw_csv(MUCA_RAW_FILES{s}, RAW_VALID_MAX);
    n_pts_covered = 0;
    for true_pt = 1:19
        code_id = TRUE_TO_CODE(true_pt);
        rel_idx = find(strcmp(rel_surface, SURFACE_NAMES{s}) & rel_point == true_pt, 1);
        has_fit = ~isempty(rel_idx) && rel_n(rel_idx) >= 2 && ~isnan(rel_slope(rel_idx));

        mask = (d.ur5_point == code_id) & (d.round_idx >= 0) & (d.round_idx <= N_ITERATIONS - 1) & ...
               (strcmp(d.phase, 'hold') | strcmp(d.phase, 'press') | strcmp(d.phase, 'retract'));
        idx = find(mask);
        if isempty(idx)
            continue;
        end

        raw_vals   = d.raw_cells(idx, code_id);
        force_vals = d.load_cell_N(idx);

        if has_fit
            a = rel_slope(rel_idx);
            b = rel_int(rel_idx);
            cp_vals = a * force_vals + b;
            n_pts_covered = n_pts_covered + 1;
        else
            cp_vals = nan(size(force_vals));
        end

        for i = 1:numel(idx)
            if isnan(cp_vals(i))
                cp_str = '';
                r2_str = '';
                n_str  = '';
            else
                cp_str = sprintf('%.5f', cp_vals(i));
                r2_str = sprintf('%.4g', rel_r2(rel_idx));
                n_str  = sprintf('%d', rel_n(rel_idx));
            end
            fprintf(out_fid, '%s,%d,%d,%s,%.4f,%.4f,%s,%s,%s\n', SURFACE_NAMES{s}, true_pt, ...
                d.round_idx(idx(i)), d.phase{idx(i)}, raw_vals(i), force_vals(i), cp_str, r2_str, n_str);
        end

        hold_mask = mask & strcmp(d.phase, 'hold');
        if has_fit && any(hold_mask)
            force_h = mean(d.load_cell_N(hold_mask), 'omitnan');
            Cp_hexmap(true_pt, s) = a * force_h + b;
            r2_hexmap(true_pt, s) = rel_r2(rel_idx);
        end
    end
    n_covered   = n_covered + n_pts_covered;
    n_uncovered = n_uncovered + (19 - n_pts_covered);
    fprintf('  %-7s: %2d/19 points have a per-point Force<->Cp_pF fit (>=2 LCR depths) -- applied per-sample\n', ...
        SURFACE_NAMES{s}, n_pts_covered);
end
fclose(out_fid);
fprintf('\n  TOTAL: %d/%d point-surface combinations covered by a real per-point fit\n', ...
    n_covered, n_covered + n_uncovered);
fprintf('  Saved: %s\n', fullfile(RESULTS_DIR, 'mucaboard_data_raw_in_pF_per_point_force.csv'));

%% ---- hex-map: estimated Cp, one panel per surface ------------------------
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
    ylabel(cb, 'Estimated Cp (pF), per-point Force fit, hold-phase mean');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
    end
    fig_suptitle(fig, 'mucaboard\_data\_raw -- estimated Cp\_pF (per-point Force regression, uncovered points grey)');
    save_fig(fig, fullfile(RESULTS_DIR, 'mucaboard_per_point_force_hexmap'));
    fprintf('  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR, 'mucaboard_per_point_force_hexmap'));
end

%% ---- hex-map: R2 of the fit used, so coverage/quality is visible at a glance
valid_r2 = r2_hexmap(~isnan(r2_hexmap));
if ~isempty(valid_r2)
    vmin = 0; vmax = 1;
    cmap = get_cmap();
    fig2 = figure('Position', [100 100 1500 550]);
    axes_list = cell(1, 3);
    for s = 1:3
        ax = axes('Parent', fig2, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
        axes_list{s} = ax;
        draw_hex_panel(ax, (1:19)', point_xy_true, r2_hexmap(:, s), cmap, vmin, vmax, ...
            8.0 / sqrt(3), upper(SURFACE_NAMES{s}));
    end
    colormap(fig2, cmap);
    for s = 1:3
        set(axes_list{s}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, 'Force<->Cp_pF fit quality (R2), per point');
    for s = 1:3
        set(axes_list{s}, 'Position', [panel_left(s), 0.12, panel_width, 0.72]);
    end
    fig_suptitle(fig2, 'Per-point Force<->Cp\_pF fit quality (R2) -- grey = no coverage (<2 LCR depths)');
    save_fig(fig2, fullfile(RESULTS_DIR, 'mucaboard_per_point_force_fit_quality_hexmap'));
    fprintf('  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR, 'mucaboard_per_point_force_fit_quality_hexmap'));
end

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('  DONE.\n');
fprintf('%s\n', repmat('=', 1, 74));
