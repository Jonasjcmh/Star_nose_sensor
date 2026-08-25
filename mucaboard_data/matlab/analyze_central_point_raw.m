% analyze_central_point_raw.m — RAW + calibration + self-normalized hex
% maps for the "raw data central point" dataset (point 10 only, 10
% iterations, all 3 surfaces)
% =========================================================================
% This dataset's CSV layout (read_muca_csv_central.m) is DIFFERENT from
% every other mucaboard log in this repo: its cell_1..19 columns are RAW
% touch-controller counts (NOT the [0,1] normalized V), and it additionally
% logs calib_1..19 -- the per-cell baseline, constant for the whole
% session. Confirmed empirically: cell_i sits in the same small-integer
% range as calib_i and rises sharply on 'hold' vs 'locate' for the pressed
% point's own column. No other file in this project logs a real baseline
% (see README_signal_normalization.md section 5) -- this one does, so it's
% the only dataset where you can apply the sensor's own normalization
% formula yourself and get a genuine (not partial-inverse) result.
%
% Produces, for point 10 (the only point pressed in this dataset), 3
% surfaces:
%   1. RAW hex maps    -- release (locate-phase) and pressed (hold-phase)
%                          raw counts, median of 10 iterations
%   2. CALIBRATION hex map -- calib_i, session-constant (one map; it does
%                          not differ between release/press by construction)
%   3. NORMALIZED hex maps -- V0/Press/deltaV computed by applying
%                          V = clip((raw-calib)/SENSITIVITY, 0, 1)^GAMMA
%                          yourself, per iteration, then median-reduced --
%                          the same formula and same per-iteration-first
%                          methodology as example_own_v0_hold_calculations.m,
%                          just fed real reconstructed raw+baseline instead
%                          of a pre-normalized log column.
%
% Usage
% -----
%   octave-cli analyze_central_point_raw.m
%   (or open/run in MATLAB)

clear; close all; clc;

% ── CONFIG ───────────────────────────────────────────────────────────────
HERE        = fileparts(mfilename('fullpath'));
DATA_DIR    = fullfile(HERE, '..', 'raw data central point');
RESULTS_DIR = fullfile(HERE, 'results');
if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end

FLAT_CSV   = fullfile(DATA_DIR, '10_iterations_5mm_flat_session_20260819_173407.csv');
HOLLOW_CSV = fullfile(DATA_DIR, '10_iterations_5mm_hollow_dome_session_20260819_165646.csv');
SOLID_CSV  = fullfile(DATA_DIR, '10_iterations_5mm_solid_session_20260819_171044.csv');

POINT_ID  = 10;    % the only point pressed in this dataset (the board's
                    % geometric center, per muca_layout.m's point_xy)
MAX_ROUND = 10;
DEPTH_MM  = 10;    % locate/press/hold/retract/post cycle presses to 10mm

% Sensor normalization constants (Integration_2/sensor.py, _normalise()) --
% see README_signal_normalization.md for the derivation.
SENSITIVITY = 30.0;
GAMMA       = 0.5;

SURFACE_NAMES = {'flat', 'solid', 'hollow'};
SURFACE_FILES = {FLAT_CSV, SOLID_CSV, HOLLOW_CSV};
[point_ids, point_xy, own_cell_col] = muca_layout();

% ── Load ─────────────────────────────────────────────────────────────────
data = struct();
for s = 1:3
    sname = SURFACE_NAMES{s};
    fprintf('Loading %-8s: %s\n', sname, SURFACE_FILES{s});
    data.(sname) = read_muca_csv_central(SURFACE_FILES{s});
end

% ── Per-surface: raw V0/Press (per-iteration, all 19 points), calib ─────
raw_v0_by_surf    = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
raw_press_by_surf = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
calib_by_surf     = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
norm_v0_by_surf    = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
norm_press_by_surf = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
norm_delta_by_surf = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));

fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('  RAW + CALIBRATION + SELF-NORMALIZED, per surface (point %d)\n', POINT_ID);
fprintf('%s\n', repmat('=', 1, 78));

for s = 1:3
    sname = SURFACE_NAMES{s};
    d = data.(sname);
    seg = muca_extract_point_segments(d, POINT_ID, own_cell_col, MAX_ROUND, DEPTH_MM);

    rounds_here = unique([seg.released.round_idx; seg.pressed.round_idx])';
    raw_v0_iter    = nan(numel(rounds_here), 19);
    raw_press_iter = nan(numel(rounds_here), 19);
    for ri = 1:numel(rounds_here)
        r = rounds_here(ri);
        raw_v0_iter(ri, :)    = mean(seg.released.values(seg.released.round_idx == r, :), 1);
        raw_press_iter(ri, :) = mean(seg.pressed.values(seg.pressed.round_idx == r, :), 1);
    end

    % RAW hex map values: median of the per-iteration raw means.
    raw_v0_by_surf.(sname)    = median(raw_v0_iter, 1)';
    raw_press_by_surf.(sname) = median(raw_press_iter, 1)';

    % Calibration, remapped from raw CSV column order to physical-point
    % order (same convention as muca_extract_point_segments.m's remap).
    calib_by_surf.(sname) = d.calib(own_cell_col)';

    % Self-normalization: apply the sensor's own formula PER ITERATION
    % (matches this project's established per-iteration-first convention --
    % see example_own_v0_hold_calculations.m), using the real calib_i as
    % baseline, THEN median-reduce across the 10 iterations.
    calib_row = calib_by_surf.(sname)';   % 1x19, broadcasts against each iter row
    ratio_v0    = min(max((raw_v0_iter    - calib_row) / SENSITIVITY, 0), 1);
    ratio_press = min(max((raw_press_iter - calib_row) / SENSITIVITY, 0), 1);
    norm_v0_iter    = ratio_v0    .^ GAMMA;
    norm_press_iter = ratio_press .^ GAMMA;

    norm_v0_by_surf.(sname)    = median(norm_v0_iter, 1)';
    norm_press_by_surf.(sname) = median(norm_press_iter, 1)';
    norm_delta_by_surf.(sname) = median(norm_press_iter - norm_v0_iter, 1)';

    fprintf('%-8s : %d iterations. own-point (P%d) raw release=%.2f raw hold=%.2f calib=%.2f -> V0=%.4f Press=%.4f\n', ...
        sname, numel(rounds_here), POINT_ID, ...
        raw_v0_by_surf.(sname)(POINT_ID), raw_press_by_surf.(sname)(POINT_ID), calib_by_surf.(sname)(POINT_ID), ...
        norm_v0_by_surf.(sname)(POINT_ID), norm_press_by_surf.(sname)(POINT_ID));
end

% ── Console + CSV table, all 19 points ──────────────────────────────────
csv_path = fullfile(RESULTS_DIR, 'central_point_raw_and_normalized.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, 'surface,point,calib,raw_v0,raw_press,norm_v0,norm_press,norm_deltaV\n');
fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('  %-8s %6s %8s %10s %10s %10s %10s %10s\n', ...
    'surface', 'point', 'calib', 'raw_V0', 'raw_Press', 'V0', 'Press', 'deltaV');
for s = 1:3
    sname = SURFACE_NAMES{s};
    for p = 1:19
        fprintf('  %-8s %6d %8.2f %10.2f %10.2f %10.4f %10.4f %10.4f\n', sname, p, ...
            calib_by_surf.(sname)(p), raw_v0_by_surf.(sname)(p), raw_press_by_surf.(sname)(p), ...
            norm_v0_by_surf.(sname)(p), norm_press_by_surf.(sname)(p), norm_delta_by_surf.(sname)(p));
        fprintf(fid, '%s,%d,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f\n', sname, p, ...
            calib_by_surf.(sname)(p), raw_v0_by_surf.(sname)(p), raw_press_by_surf.(sname)(p), ...
            norm_v0_by_surf.(sname)(p), norm_press_by_surf.(sname)(p), norm_delta_by_surf.(sname)(p));
    end
end
fclose(fid);
fprintf('\n  saved: %s\n', csv_path);

% ── Plot: RAW + calibration, shared scale (same physical unit: raw counts) ──
raw_all = [];
for s = 1:3
    sname = SURFACE_NAMES{s};
    raw_all = [raw_all; calib_by_surf.(sname); raw_v0_by_surf.(sname); raw_press_by_surf.(sname)]; %#ok<AGROW>
end
RAW_CLIM = [min(raw_all), max(raw_all)];
fprintf('\nShared color scale across calib/raw_V0/raw_Press (raw counts): [%.2f, %.2f]\n', RAW_CLIM(1), RAW_CLIM(2));

fig_calib = muca_plot_hex_comparison(calib_by_surf, ...
    'Calibration baseline (raw counts, session-constant)', ...
    'Muca-Board -- per-cell calibration baseline, central-point dataset', POINT_ID, RAW_CLIM);
save_fig(fig_calib, fullfile(RESULTS_DIR, 'central_calib_hexmap'));

fig_rawv0 = muca_plot_hex_comparison(raw_v0_by_surf, ...
    'Raw V0 (release), median of 10 iterations (raw counts)', ...
    sprintf('Muca-Board -- raw release counts while P%02d indented, central-point dataset', POINT_ID), POINT_ID, RAW_CLIM);
save_fig(fig_rawv0, fullfile(RESULTS_DIR, 'central_raw_v0_hexmap'));

fig_rawpress = muca_plot_hex_comparison(raw_press_by_surf, ...
    'Raw Press (hold), median of 10 iterations (raw counts)', ...
    sprintf('Muca-Board -- raw hold counts while P%02d indented, central-point dataset', POINT_ID), POINT_ID, RAW_CLIM);
save_fig(fig_rawpress, fullfile(RESULTS_DIR, 'central_raw_press_hexmap'));

% ── Plot: self-NORMALIZED (V computed from raw+calib, [0,1]-ish) ────────
norm_all = [];
for s = 1:3
    sname = SURFACE_NAMES{s};
    norm_all = [norm_all; norm_v0_by_surf.(sname); norm_press_by_surf.(sname); norm_delta_by_surf.(sname)]; %#ok<AGROW>
end
NORM_CLIM = [min(norm_all), max(norm_all)];
fprintf('Shared color scale across normalized V0/Press/deltaV: [%.4f, %.4f]\n', NORM_CLIM(1), NORM_CLIM(2));

fig_normv0 = muca_plot_hex_comparison(norm_v0_by_surf, ...
    'V0 (release), self-normalized from raw+calib, median of 10 iter', ...
    sprintf('Muca-Board -- self-normalized V0 while P%02d indented, central-point dataset', POINT_ID), POINT_ID, NORM_CLIM);
save_fig(fig_normv0, fullfile(RESULTS_DIR, 'central_norm_v0_hexmap'));

fig_normpress = muca_plot_hex_comparison(norm_press_by_surf, ...
    'Press (hold), self-normalized from raw+calib, median of 10 iter', ...
    sprintf('Muca-Board -- self-normalized Press while P%02d indented, central-point dataset', POINT_ID), POINT_ID, NORM_CLIM);
save_fig(fig_normpress, fullfile(RESULTS_DIR, 'central_norm_press_hexmap'));

fig_normdelta = muca_plot_hex_comparison(norm_delta_by_surf, ...
    'deltaV = Press - V0, self-normalized, median of 10 iter', ...
    sprintf('Muca-Board -- self-normalized deltaV while P%02d indented, central-point dataset', POINT_ID), POINT_ID, NORM_CLIM);
save_fig(fig_normdelta, fullfile(RESULTS_DIR, 'central_norm_deltaV_hexmap'));

fprintf('\nFigures + CSV saved in: %s/\n', RESULTS_DIR);
