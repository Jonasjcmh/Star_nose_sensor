% =========================================================================
% per_point_surface_relationships.m
%
% Answers: "for each cell (point), on each surface, what is the
% mathematical relationship between the muca board reading, the LCR
% meter's Cp_pF, and the Futek force reading?"
%
% Three signals, three pairwise relationships, but they are NOT all
% equally well-determined from the data available:
%
%   Force <-> Cp_pF   REAL regression, per point/surface. Capacitance_measurement
%                      sweeps each point across several depths (=several
%                      forces), giving multiple (Force, Cp_pF) samples per
%                      point -- enough to fit a genuine line, Cp_pF = a_F *
%                      Force + b_F, with its own R^2.
%
%   Force <-> muca V   NOT fittable per point. mucaboard_data_raw presses
%                      each point at only ONE depth (10mm), so there is
%                      only a SINGLE (Force, V) sample per point -- a
%                      point, not a line. Reported as-is (no slope/R^2).
%
%   muca V <-> Cp_pF   Same limitation as above for a PER-POINT slope
%                      (only one V sample per point). What IS reported:
%                      (a) Cp_pF_interpolated -- the Force<->Cp_pF fit
%                      evaluated at this point's own Force (a single,
%                      point-specific "best guess" real capacitance), and
%                      (b) Cp_pF_from_pooled_fit -- what
%                      calibrate_capacitance_muca_lcr_futek.m's STEP B
%                      POOLED (across all 19 points) V->Cp_pF fit predicts
%                      for this point's own V. The gap between (a) and (b)
%                      is exactly how much error the pooled/global model
%                      makes for THIS specific point -- i.e. how good a
%                      single per-surface equation actually is, point by
%                      point.
%
% Output: results/per_point_surface_relationships.csv (machine-readable)
% and results/per_point_surface_relationships.md (human-readable, one
% table per surface, includes each point's Force<->Cp_pF equation).
%
% Run AFTER calibrate_capacitance_muca_lcr_futek.m (reuses its shared
% helper files, which is why they live in their own .m files now).
% =========================================================================

clear; close all; clc;

function s = fmt_or_na(v, spec)
% Octave requires local script-functions defined before first use.
    if isnan(v)
        s = 'n/a';
    else
        s = sprintf(spec, v);
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
addpath(MUCA_TOOLKIT);   % read_lcr_csv / load_lcr_files / lcr_stats_by_point_depth

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
fprintf('  PER-POINT, PER-SURFACE RELATIONSHIPS: muca reading <-> LCR Cp_pF <-> Force\n');
fprintf('%s\n', repmat('=', 1, 74));

% ---- output containers ---------------------------------------------------
csv_rows = {};   % each row: cellstr of the CSV fields, appended as we go
md_lines = {};

md_lines{end+1} = '# Per-point, per-surface relationships: muca reading <-> LCR Cp_pF <-> Force';
md_lines{end+1} = '';
md_lines{end+1} = ['Point ids are TRUE board labels (matching the physical board''s own numbering,' ...
    ' see `mucaboard_data_raw/matlab/simple_19points_analysis.m` for the derivation).'];
md_lines{end+1} = '';
md_lines{end+1} = ['`Force<->Cp_pF` is a REAL per-point linear regression (Capacitance_measurement''s' ...
    ' own multi-depth sweep at that point). `Force<->V` has only ONE sample per point in' ...
    ' mucaboard_data_raw (single depth = 10mm), so no slope is fittable there -- the single' ...
    ' matched (Force, V) pair is reported instead. `Cp_pF (interp)` evaluates the point''s own' ...
    ' Force<->Cp_pF fit at its matched Force; `Cp_pF (pooled V-fit)` evaluates the SURFACE-WIDE' ...
    ' pooled V->Cp_pF fit (from `calibrate_capacitance_muca_lcr_futek.m`, STEP B) at this' ...
    ' point''s own V -- the difference between the two is that pooled model''s point-specific error.'];
md_lines{end+1} = '';

for s = 1:3
    fprintf('\n-- %s --\n', upper(SURFACE_NAMES{s}));

    % ---- mucaboard_data_raw: single (Force, raw, V) sample per point ----
    d = read_muca_raw_csv(MUCA_RAW_FILES{s}, RAW_VALID_MAX);
    muca_rows = muca_raw_own_reading_by_point(d, TRUE_TO_CODE, N_ITERATIONS, SENSITIVITY, GAMMA);
    % muca_rows: [true_point, force_N, raw_own, V_own]

    % ---- Capacitance_measurement: per-point (Force, Cp_pF) depth sweep --
    lcr_d = load_lcr_files(LCR_FILES{s});
    lcr_rows = lcr_stats_by_point_depth(lcr_d);   % [point(code), depth_mm, force_N, Cp_pF]

    % ---- pooled V->Cp_pF fit for this surface (same as STEP B) ----------
    matched_v = [];
    matched_cp = [];
    for true_pt = 1:19
        if isnan(muca_rows(true_pt, 1)), continue; end
        code_id = TRUE_TO_CODE(true_pt);
        sub = lcr_rows(lcr_rows(:, 1) == code_id, [3 4]);
        if isempty(sub), continue; end
        [cp_est, ~] = interp_force_to_cp(sub, muca_rows(true_pt, 2));
        if isnan(cp_est), continue; end
        matched_v(end+1)  = muca_rows(true_pt, 4); %#ok<AGROW>
        matched_cp(end+1) = cp_est; %#ok<AGROW>
    end
    if numel(matched_v) >= 2
        pooled = polyfit(matched_v, matched_cp, 1);
    else
        pooled = [NaN NaN];
    end
    fprintf('  Pooled V->Cp_pF fit (this surface): Cp_pF = %.4f*V + %.4f  (n=%d)\n', ...
        pooled(1), pooled(2), numel(matched_v));

    md_lines{end+1} = sprintf('## %s', upper(SURFACE_NAMES{s}));
    md_lines{end+1} = '';
    md_lines{end+1} = sprintf('Pooled (surface-wide) fit: `Cp_pF = %.4f * V_own + %.4f` (n=%d points)', ...
        pooled(1), pooled(2), numel(matched_v));
    md_lines{end+1} = '';
    md_lines{end+1} = ['| point | code_id | Force<->Cp_pF fit (per point) | R2 | n | Force_N | ' ...
        'muca_raw | muca_V | Cp_pF (interp) | Cp_pF (pooled V-fit) | residual (pF) |'];
    md_lines{end+1} = '|---|---|---|---|---|---|---|---|---|---|---|';

    for true_pt = 1:19
        code_id = TRUE_TO_CODE(true_pt);
        sub = lcr_rows(lcr_rows(:, 1) == code_id, :);   % [point, depth_mm, force_N, Cp_pF]

        % Force<->Cp_pF: real per-point regression from the LCR sweep
        if size(sub, 1) >= 2
            pf = polyfit(sub(:, 3), sub(:, 4), 1);
            yfit = polyval(pf, sub(:, 3));
            ss_res = sum((sub(:, 4) - yfit).^2);
            ss_tot = sum((sub(:, 4) - mean(sub(:, 4))).^2);
            r2f = 1 - ss_res / max(ss_tot, eps);
            nf = size(sub, 1);
            eq_str = sprintf('Cp_pF = %.4f*F + %.4f', pf(1), pf(2));
        else
            pf = [NaN NaN];
            r2f = NaN;
            nf = size(sub, 1);
            eq_str = 'n/a (fewer than 2 depths for this point)';
        end

        if isnan(muca_rows(true_pt, 1))
            force_muca = NaN; raw_own = NaN; v_own = NaN;
        else
            force_muca = muca_rows(true_pt, 2);
            raw_own    = muca_rows(true_pt, 3);
            v_own      = muca_rows(true_pt, 4);
        end

        if ~isnan(force_muca) && ~any(isnan(pf))
            cp_interp = polyval(pf, force_muca);
        elseif ~isempty(sub) && ~isnan(force_muca)
            cp_interp = interp_force_to_cp(sub(:, [3 4]), force_muca);
        else
            cp_interp = NaN;
        end

        if ~isnan(v_own) && ~any(isnan(pooled))
            cp_pooled = polyval(pooled, v_own);
        else
            cp_pooled = NaN;
        end

        residual = cp_interp - cp_pooled;

        csv_rows(end+1, :) = { SURFACE_NAMES{s}, true_pt, code_id, ...
            sprintf('%.6g', pf(1)), sprintf('%.6g', pf(2)), sprintf('%.4g', r2f), nf, ...
            sprintf('%.4g', force_muca), sprintf('%.4g', raw_own), sprintf('%.5g', v_own), ...
            sprintf('%.5g', cp_interp), sprintf('%.5g', cp_pooled), sprintf('%.5g', residual) }; %#ok<AGROW>

        md_lines{end+1} = sprintf('| P%02d | %d | %s | %s | %d | %s | %s | %s | %s | %s | %s |', ...
            true_pt, code_id, eq_str, fmt_or_na(r2f, '%.3f'), nf, ...
            fmt_or_na(force_muca, '%.3f'), fmt_or_na(raw_own, '%.2f'), fmt_or_na(v_own, '%.4f'), ...
            fmt_or_na(cp_interp, '%.4f'), fmt_or_na(cp_pooled, '%.4f'), fmt_or_na(residual, '%+.4f'));
    end
    md_lines{end+1} = '';
end

% ---- write CSV ------------------------------------------------------------
csv_path = fullfile(RESULTS_DIR, 'per_point_surface_relationships.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, ['surface,true_point,code_id,force_cp_slope,force_cp_intercept,force_cp_r2,' ...
    'force_cp_n,force_N,muca_raw,muca_V,Cp_pF_interp,Cp_pF_pooled_fit,residual_pF\n']);
for i = 1:size(csv_rows, 1)
    fprintf(fid, '%s,%d,%d,%s,%s,%s,%d,%s,%s,%s,%s,%s,%s\n', csv_rows{i, :});
end
fclose(fid);
fprintf('\nSaved: %s\n', csv_path);

% ---- write Markdown table --------------------------------------------------
md_path = fullfile(RESULTS_DIR, 'per_point_surface_relationships.md');
fid = fopen(md_path, 'w');
fprintf(fid, '%s\n', strjoin(md_lines, '\n'));
fclose(fid);
fprintf('Saved: %s\n', md_path);

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('  DONE.\n');
fprintf('%s\n', repmat('=', 1, 74));
