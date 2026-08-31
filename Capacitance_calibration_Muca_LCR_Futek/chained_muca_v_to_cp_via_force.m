% =========================================================================
% chained_muca_v_to_cp_via_force.m
%
% Builds a genuine PER-POINT muca_V <-> Cp_pF relationship -- something
% every earlier script in this folder couldn't do directly, because
% mucaboard_data_raw only presses each point at ONE fixed depth (10mm), so
% there was only ever a single (V, Force) SAMPLE per point, not enough to
% fit a per-point slope.
%
% The fix: mucaboard_data_raw's 'press' and 'retract' phases are the robot
% CONTINUOUSLY moving from 0->10mm and 10->0mm. During that ramp, BOTH the
% muca raw reading AND the Futek force sweep through a wide range of
% values, sample by sample, all for the SAME point -- e.g. point 1 on flat
% alone has its force sweep ~2.6N to ~19.5N with the raw reading rising
% from ~22 to ~40 in step, just within one press+retract pass. Pooled
% across all 10 iterations' press+hold+retract phases, that is hundreds of
% real (Force, V) samples per point -- enough to fit a genuine per-point
% Force<->V regression using ONLY mucaboard_data_raw's own data.
%
% Chain: muca_V --[per-point Force<->V fit, THIS script, from
% mucaboard_data_raw's own ramp]--> Force --[per-point Force<->Cp_pF fit,
% per_point_surface_relationships.m, from Capacitance_measurement's LCR
% sweep, R^2 0.97-0.99]--> Cp_pF
%
% Both links are per-point, both are fit from real multi-sample sweeps (not
% pooled across points, not borrowed from another surface) -- this is the
% most direct route from muca_V to Cp_pF available in this project's data.
%
% Run AFTER per_point_surface_relationships.m (reads its output CSV for the
% Force<->Cp_pF half of the chain).
% =========================================================================

clear; close all; clc;

HERE          = fileparts(mfilename('fullpath'));
REPO_ROOT     = fullfile(HERE, '..');
MUCA_TOOLKIT  = fullfile(REPO_ROOT, 'mucaboard_data', 'matlab');
MUCA_RAW_LOGS = fullfile(REPO_ROOT, 'mucaboard_data_raw', 'logs');
RESULTS_DIR   = fullfile(HERE, 'results');
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

fprintf('%s\n', repmat('=', 1, 74));
fprintf('  CHAINED muca_V -> Force -> Cp_pF (both links per-point, both from real sweeps)\n');
fprintf('%s\n', repmat('=', 1, 74));

%% ---- load the Force<->Cp_pF half of the chain (already computed) -------
rel_csv = fullfile(RESULTS_DIR, 'per_point_surface_relationships.csv');
if ~exist(rel_csv, 'file')
    error('chained_muca_v_to_cp_via_force:missing', ...
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

%% ---- build the Force<->V half of the chain, per point, per surface -----
fprintf('\n[STEP 1] Fit per-point Force<->V_own from mucaboard_data_raw''s own press/retract ramp\n');
fprintf('%s\n', repmat('-', 1, 74));

csv_rows = {};
md_lines = {};
md_lines{end+1} = '# Chained muca_V -> Force -> Cp_pF (both links fit per point, from real sweeps)';
md_lines{end+1} = '';
md_lines{end+1} = ['`Force<->V` is fit HERE, per point, from mucaboard_data_raw''s own press+hold+retract' ...
    ' phase samples (the robot''s continuous 0->10mm ramp gives many (Force,V) pairs per point, not' ...
    ' just the single hold-phase mean used everywhere else in this folder). `Force<->Cp_pF` is the' ...
    ' per-point fit already computed by per_point_surface_relationships.m (from Capacitance_measurement''s' ...
    ' real multi-depth LCR sweep, R2 0.97-0.99). `Cp_pF at V=1` and `Cp_pF at V=0` show the chained' ...
    ' model''s prediction at the extremes of the normalized reading, for a quick sanity read.'];
md_lines{end+1} = '';

fv_fits = cell(19, 3);   % [slope, intercept, r2, n] per (point, surface)

for s = 1:3
    fprintf('\n  -- %s --\n', upper(SURFACE_NAMES{s}));
    d = read_muca_raw_csv(MUCA_RAW_FILES{s}, RAW_VALID_MAX);

    md_lines{end+1} = sprintf('## %s', upper(SURFACE_NAMES{s}));
    md_lines{end+1} = '';
    md_lines{end+1} = ['| point | code_id | Force<->V fit (from ramp) | R2(F<->V) | n(F<->V) | ' ...
        'Force<->Cp_pF fit | R2(F<->Cp) | n(F<->Cp) | Cp_pF at V=0 | Cp_pF at V=1 |'];
    md_lines{end+1} = '|---|---|---|---|---|---|---|---|---|---|';

    n_chained = 0;
    for true_pt = 1:19
        code_id = TRUE_TO_CODE(true_pt);
        mask = (d.ur5_point == code_id) & (d.round_idx >= 0) & (d.round_idx <= N_ITERATIONS - 1) & ...
               (strcmp(d.phase, 'press') | strcmp(d.phase, 'hold') | strcmp(d.phase, 'retract'));
        idx = find(mask);
        if numel(idx) < 5
            fv_fits{true_pt, s} = [NaN NaN NaN 0];
            md_lines{end+1} = sprintf('| P%02d | %d | n/a (fewer than 5 ramp samples) | n/a | %d | n/a | n/a | n/a | n/a | n/a |', ...
                true_pt, code_id, numel(idx));
            continue;
        end

        calib = d.calib_raw(code_id);
        raw_vals = d.raw_cells(idx, code_id);
        force_vals = d.load_cell_N(idx);
        ratio = min(max((raw_vals - calib) / SENSITIVITY, 0), 1);
        v_vals = ratio .^ GAMMA;

        valid = ~isnan(v_vals) & ~isnan(force_vals);
        v_vals = v_vals(valid);
        force_vals = force_vals(valid);
        if numel(v_vals) < 5
            fv_fits{true_pt, s} = [NaN NaN NaN numel(v_vals)];
            continue;
        end

        pfv = polyfit(force_vals, v_vals, 1);
        yfit = polyval(pfv, force_vals);
        r2fv = 1 - sum((v_vals - yfit).^2) / max(sum((v_vals - mean(v_vals)).^2), eps);
        fv_fits{true_pt, s} = [pfv(1), pfv(2), r2fv, numel(v_vals)];

        % invert: V -> Force -> Cp_pF requires inverting V = a*F + b -> F = (V-b)/a
        a_fv = pfv(1); b_fv = pfv(2);

        rel_idx = find(strcmp(rel_surface, SURFACE_NAMES{s}) & rel_point == true_pt, 1);
        has_cp_fit = ~isempty(rel_idx) && rel_n(rel_idx) >= 2 && ~isnan(rel_slope(rel_idx));

        if has_cp_fit && abs(a_fv) > eps
            a_fc = rel_slope(rel_idx);
            b_fc = rel_int(rel_idx);
            % Cp_pF(V) = a_fc * ((V - b_fv)/a_fv) + b_fc
            cp_at_v0 = a_fc * ((0 - b_fv) / a_fv) + b_fc;
            cp_at_v1 = a_fc * ((1 - b_fv) / a_fv) + b_fc;
            n_chained = n_chained + 1;
            eq_fv = sprintf('V = %.5f*F + %.4f', a_fv, b_fv);
            eq_fc = sprintf('Cp_pF = %.4f*F + %.4f', a_fc, b_fc);
            md_lines{end+1} = sprintf('| P%02d | %d | %s | %.3f | %d | %s | %.3f | %d | %.4f | %.4f |', ...
                true_pt, code_id, eq_fv, r2fv, numel(v_vals), eq_fc, rel_r2(rel_idx), rel_n(rel_idx), ...
                cp_at_v0, cp_at_v1);
            csv_rows(end+1, :) = { SURFACE_NAMES{s}, true_pt, code_id, ...
                sprintf('%.6g', a_fv), sprintf('%.6g', b_fv), sprintf('%.4g', r2fv), numel(v_vals), ...
                sprintf('%.6g', a_fc), sprintf('%.6g', b_fc), sprintf('%.4g', rel_r2(rel_idx)), rel_n(rel_idx), ...
                sprintf('%.5g', cp_at_v0), sprintf('%.5g', cp_at_v1) }; %#ok<AGROW>
        else
            eq_fv = sprintf('V = %.5f*F + %.4f', a_fv, b_fv);
            md_lines{end+1} = sprintf('| P%02d | %d | %s | %.3f | %d | n/a (no Force<->Cp fit) | n/a | n/a | n/a | n/a |', ...
                true_pt, code_id, eq_fv, r2fv, numel(v_vals));
        end
    end
    fprintf('  %d/19 points got a full chained V->Cp_pF model\n', n_chained);
    md_lines{end+1} = '';
end

%% ---- write outputs --------------------------------------------------------
csv_path = fullfile(RESULTS_DIR, 'chained_muca_v_to_cp_via_force.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, ['surface,true_point,code_id,force_v_slope,force_v_intercept,force_v_r2,force_v_n,' ...
    'force_cp_slope,force_cp_intercept,force_cp_r2,force_cp_n,Cp_pF_at_V0,Cp_pF_at_V1\n']);
for i = 1:size(csv_rows, 1)
    fprintf(fid, '%s,%d,%d,%s,%s,%s,%d,%s,%s,%s,%d,%s,%s\n', csv_rows{i, :});
end
fclose(fid);
fprintf('\nSaved: %s\n', csv_path);

md_path = fullfile(RESULTS_DIR, 'chained_muca_v_to_cp_via_force.md');
fid = fopen(md_path, 'w');
fprintf(fid, '%s\n', strjoin(md_lines, '\n'));
fclose(fid);
fprintf('Saved: %s\n', md_path);

%% ---- apply the chained model to every sample of mucaboard_data_raw -----
fprintf('\n[STEP 2] Applying the chained model to every mucaboard_data_raw sample\n');
fprintf('%s\n', repmat('-', 1, 74));

out_fid = fopen(fullfile(RESULTS_DIR, 'mucaboard_data_raw_in_pF_chained_via_V.csv'), 'w');
fprintf(out_fid, 'surface,true_point,round_idx,phase,raw_own,muca_V,Cp_pF_estimated,chain_r2_min\n');

Cp_hexmap = nan(19, 3);
r2_hexmap = nan(19, 3);

for s = 1:3
    d = read_muca_raw_csv(MUCA_RAW_FILES{s}, RAW_VALID_MAX);
    n_pts = 0;
    for true_pt = 1:19
        code_id = TRUE_TO_CODE(true_pt);
        fit_fv = fv_fits{true_pt, s};
        rel_idx = find(strcmp(rel_surface, SURFACE_NAMES{s}) & rel_point == true_pt, 1);
        has_cp_fit = ~isempty(rel_idx) && rel_n(rel_idx) >= 2 && ~isnan(rel_slope(rel_idx));
        has_fv_fit = ~any(isnan(fit_fv(1:3))) && abs(fit_fv(1)) > eps;
        if ~has_cp_fit || ~has_fv_fit
            continue;
        end
        n_pts = n_pts + 1;
        a_fv = fit_fv(1); b_fv = fit_fv(2); r2_fv = fit_fv(3);
        a_fc = rel_slope(rel_idx); b_fc = rel_int(rel_idx); r2_fc = rel_r2(rel_idx);
        chain_r2 = min(r2_fv, r2_fc);

        mask = (d.ur5_point == code_id) & (d.round_idx >= 0) & (d.round_idx <= N_ITERATIONS - 1) & ...
               (strcmp(d.phase, 'hold') | strcmp(d.phase, 'press') | strcmp(d.phase, 'retract'));
        idx = find(mask);
        if isempty(idx), continue; end

        calib = d.calib_raw(code_id);
        raw_vals = d.raw_cells(idx, code_id);
        ratio = min(max((raw_vals - calib) / SENSITIVITY, 0), 1);
        v_vals = ratio .^ GAMMA;
        force_est = (v_vals - b_fv) / a_fv;
        cp_vals = a_fc * force_est + b_fc;

        for i = 1:numel(idx)
            fprintf(out_fid, '%s,%d,%d,%s,%.4f,%.5f,%.5f,%.4g\n', SURFACE_NAMES{s}, true_pt, ...
                d.round_idx(idx(i)), d.phase{idx(i)}, raw_vals(i), v_vals(i), cp_vals(i), chain_r2);
        end

        hold_mask = mask & strcmp(d.phase, 'hold');
        if any(hold_mask)
            raw_h = d.raw_cells(hold_mask, code_id);
            ratio_h = min(max((raw_h - calib) / SENSITIVITY, 0), 1);
            v_h = mean(ratio_h .^ GAMMA, 'omitnan');
            force_h = (v_h - b_fv) / a_fv;
            Cp_hexmap(true_pt, s) = a_fc * force_h + b_fc;
            r2_hexmap(true_pt, s) = chain_r2;
        end
    end
    fprintf('  %-7s: %2d/19 points got the chained V->Force->Cp_pF model applied\n', SURFACE_NAMES{s}, n_pts);
end
fclose(out_fid);
fprintf('\n  Saved: %s\n', fullfile(RESULTS_DIR, 'mucaboard_data_raw_in_pF_chained_via_V.csv'));

%% ---- hex maps --------------------------------------------------------------
[point_ids, point_xy, ~] = muca_layout();
point_xy_true = point_xy(TRUE_TO_CODE, :);

valid_vals = Cp_hexmap(~isnan(Cp_hexmap));
if ~isempty(valid_vals)
    vmin = min(valid_vals); vmax = max(valid_vals);
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
    for s = 1:3, set(axes_list{s}, 'CLim', [vmin, vmax]); end
    cb = colorbar(axes_list{3}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, 'Estimated Cp (pF), chained V->Force->Cp model, hold-phase mean');
    for s = 1:3, set(axes_list{s}, 'Position', [panel_left(s), 0.12, panel_width, 0.72]); end
    fig_suptitle(fig, 'mucaboard\_data\_raw -- estimated Cp\_pF (chained V->Force->Cp\_pF, per-point)');
    save_fig(fig, fullfile(RESULTS_DIR, 'mucaboard_chained_v_hexmap'));
    fprintf('  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR, 'mucaboard_chained_v_hexmap'));
end

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('  DONE.\n');
fprintf('%s\n', repmat('=', 1, 74));
