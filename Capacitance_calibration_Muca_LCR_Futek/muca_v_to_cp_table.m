% =========================================================================
% muca_v_to_cp_table.m
%
% Composes the direct muca_V <-> Cp_pF equation, per point, per surface,
% from chained_muca_v_to_cp_via_force.m's two chained fits:
%
%   V = a_fv*Force + b_fv          (fit from mucaboard_data_raw's own ramp)
%   Cp_pF = a_fc*Force + b_fc      (fit from Capacitance_measurement's LCR sweep)
%
% Substituting Force = (V - b_fv)/a_fv into the second equation gives ONE
% direct linear relationship:
%
%   Cp_pF = (a_fc/a_fv) * V + (b_fc - a_fc*b_fv/a_fv)
%         =      A       * V +           B
%
% i.e. exactly the equation asked for -- "the relationship between muca
% reading and capacitance, per point". Trivially recovered from the two
% points already computed at V=0 and V=1 (Cp_pF_at_V0, Cp_pF_at_V1) in
% chained_muca_v_to_cp_via_force.csv: A = Cp_pF_at_V1 - Cp_pF_at_V0,
% B = Cp_pF_at_V0 -- so this script just reads that CSV and reformats.
%
% chain_r2 (the quality indicator carried through) is the MINIMUM of the
% two links' individual R2 -- a chain is only as strong as its weakest
% link, so this is the honest quality figure for the composed equation,
% not an average that could hide a bad link.
%
% Run AFTER chained_muca_v_to_cp_via_force.m (reads its output CSV).
% =========================================================================

clear; close all; clc;

HERE        = fileparts(mfilename('fullpath'));
RESULTS_DIR = fullfile(HERE, 'results');
SURFACE_NAMES = {'flat', 'solid', 'hollow'};

src_csv = fullfile(RESULTS_DIR, 'chained_muca_v_to_cp_via_force.csv');
if ~exist(src_csv, 'file')
    error('muca_v_to_cp_table:missing', ...
        'Run chained_muca_v_to_cp_via_force.m first -- %s not found.', src_csv);
end

fid = fopen(src_csv, 'r');
header_line = fgetl(fid);
header = strsplit(header_line, ',', 'CollapseDelimiters', false);
fmt = repmat('%s', 1, numel(header));
C = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
fclose(fid);
col = @(name) C{find(strcmp(header, name), 1)};

src_surface = col('surface');
src_point   = str2double(col('true_point'));
src_code    = str2double(col('code_id'));
src_fv_r2   = str2double(col('force_v_r2'));
src_fc_r2   = str2double(col('force_cp_r2'));
src_cp_v0   = str2double(col('Cp_pF_at_V0'));
src_cp_v1   = str2double(col('Cp_pF_at_V1'));

fprintf('%s\n', repmat('=', 1, 74));
fprintf('  DIRECT muca_V <-> Cp_pF relationship, per point, per surface\n');
fprintf('%s\n', repmat('=', 1, 74));

md_lines = {};
md_lines{end+1} = '# muca reading <-> real capacitance (Cp, pF), per point, per surface';
md_lines{end+1} = '';
md_lines{end+1} = ['`Cp_pF = A * V_own + B`, where V_own is the mucaboard''s own normalized [0,1]' ...
    ' reading for that point (see mucaboard_data/README_capacitance_estimation.md section 1 for' ...
    ' how a raw count becomes V_own). Composed from two per-point fits chained through Force' ...
    ' (see chained_muca_v_to_cp_via_force.m): mucaboard_data_raw''s own press/retract ramp gives' ...
    ' Force<->V; Capacitance_measurement''s LCR sweep gives Force<->Cp_pF. `R2 (weakest link)` is' ...
    ' the MINIMUM of the two chained fits'' own R2 -- the honest quality figure for the composed' ...
    ' equation, since a chain is only as strong as its weakest link.'];
md_lines{end+1} = '';

csv_rows = {};

for s = 1:3
    fprintf('\n-- %s --\n', upper(SURFACE_NAMES{s}));
    md_lines{end+1} = sprintf('## %s', upper(SURFACE_NAMES{s}));
    md_lines{end+1} = '';
    md_lines{end+1} = '| point | code_id | Cp_pF = A*V + B | A (slope) | B (intercept) | R2 (weakest link) |';
    md_lines{end+1} = '|---|---|---|---|---|---|';

    for true_pt = 1:19
        idx = find(strcmp(src_surface, SURFACE_NAMES{s}) & src_point == true_pt, 1);
        if isempty(idx) || isnan(src_cp_v0(idx)) || isnan(src_cp_v1(idx))
            md_lines{end+1} = sprintf('| P%02d | -- | n/a | n/a | n/a | n/a |', true_pt);
            continue;
        end
        A = src_cp_v1(idx) - src_cp_v0(idx);
        B = src_cp_v0(idx);
        r2_weak = min(src_fv_r2(idx), src_fc_r2(idx));
        code_id = src_code(idx);

        eq_str = sprintf('Cp_pF = %.4f*V + %.4f', A, B);
        md_lines{end+1} = sprintf('| P%02d | %d | %s | %.4f | %.4f | %.3f |', ...
            true_pt, code_id, eq_str, A, B, r2_weak);
        csv_rows(end+1, :) = { SURFACE_NAMES{s}, true_pt, code_id, ...
            sprintf('%.6g', A), sprintf('%.6g', B), sprintf('%.4g', r2_weak) }; %#ok<AGROW>

        fprintf('  P%02d (code %2d): Cp_pF = %8.4f * V + %7.4f   R2(weakest link)=%.3f\n', ...
            true_pt, code_id, A, B, r2_weak);
    end
    md_lines{end+1} = '';
end

csv_path = fullfile(RESULTS_DIR, 'muca_v_to_cp_table.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, 'surface,true_point,code_id,slope_A,intercept_B,r2_weakest_link\n');
for i = 1:size(csv_rows, 1)
    fprintf(fid, '%s,%d,%d,%s,%s,%s\n', csv_rows{i, :});
end
fclose(fid);
fprintf('\nSaved: %s\n', csv_path);

md_path = fullfile(RESULTS_DIR, 'muca_v_to_cp_table.md');
fid = fopen(md_path, 'w');
fprintf(fid, '%s\n', strjoin(md_lines, '\n'));
fclose(fid);
fprintf('Saved: %s\n', md_path);

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('  DONE.\n');
fprintf('%s\n', repmat('=', 1, 74));
