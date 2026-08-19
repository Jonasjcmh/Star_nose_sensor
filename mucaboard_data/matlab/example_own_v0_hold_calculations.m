% example_own_v0_hold_calculations.m — V0 (release) and Press (hold) per
% indented point, kept SEPARATE (no pooling across P3/P10/P14), reduced by
% median across the 10 iterations, plotted as one 3-surface hex comparison
% per indented point per metric.
%
% Data model
% ----------
% For each surface (3) x indented point ip in {3,10,14} (3) x physical
% point j (1-19) x iteration r (0-9):
%   V0(j,r)    = mean of j's raw RELEASE-phase ('locate') samples,
%                round r of ip's press cycle
%   Press(j,r) = mean of j's raw HOLD-phase samples, round r of ip's
%                press cycle
% That's a set of 20 values per point j (10 V0's + 10 Press's) for THIS
% indented point; 19*20 = 380 per indented point; 3*380 = 1140 per
% surface; 3*1140 = 3420 total. Nothing is pooled/merged across the 3
% indented points -- each ip's (10 x 19) V0 and Press matrices are reduced
% to 19x1 via median ACROSS ITS OWN 10 ITERATIONS ONLY, then plotted as
% its own hex map (with ip outlined in red). This is deliberate: pooling
% P3/P10/P14's data together would blend a point's genuine own-press
% signal with two much weaker cross-talk signals from the other two
% points' press events -- keeping them separate avoids that entirely.
%
% Building blocks used here (all reusable on their own):
%   muca_load_three_surfaces.m     -- read_muca_csv.m for all 3 CSVs at once
%   muca_extract_point_segments.m  -- raw, UN-reduced pressed/released rows
%                                      for one indented point, all 19 cells,
%                                      tagged with round_idx
%   muca_plot_hex_comparison.m     -- 3-panel hex plot for ANY 19x1-per-
%                                      surface values you hand it
%
% Usage
% -----
%   octave-cli example_own_v0_hold_calculations.m
%   (or open/run in MATLAB, or copy this file and edit the CALC section)

clear; close all; clc;

% ── CONFIG ───────────────────────────────────────────────────────────────
% Default: the 10-round *_3points_session_* logs (points 3/10/14 only,
% single depth = 10mm per round). Swap these three lines + POINT_IDS +
% MAX_ROUND + DEPTH_MM for the *_5iters_*/long_dataset_* logs to get full
% 19-point coverage at 5 rounds instead -- see analyze_muca_v0_hold_allpoints.m
% for that exact file trio.
HERE        = fileparts(mfilename('fullpath'));
LOG_DIR     = fullfile(HERE, '..', 'logs');
RESULTS_DIR = fullfile(HERE, 'results');
if ~exist(RESULTS_DIR, 'dir')
    mkdir(RESULTS_DIR);
end

FLAT_CSV   = fullfile(LOG_DIR, 'flat_3points_session_20260813_180036.csv');
SOLID_CSV  = fullfile(LOG_DIR, 'solid_3points_session_20260813_173548.csv');
HOLLOW_CSV = fullfile(LOG_DIR, 'hollow_3points_session_20260813_181218.csv');

MAX_ROUND = 10;
DEPTH_MM  = 10;            % these logs press to one depth only; use [] to skip filtering
POINT_IDS = [10];   % the only points with full 10-round coverage in these logs

% Sensor normalization constants (Integration_2/sensor.py, _normalise()):
%   V_i = clip((raw_i - baseline_i) / SENSITIVITY, 0, 1) ^ GAMMA
% See README_signal_normalization.md for the full derivation. Only
% SENSITIVITY is needed below -- the "reverse" calculation undoes the
% GAMMA power (squaring, since GAMMA=0.5) and the /SENSITIVITY division,
% recovering (raw_i - baseline_i) -- NOT the absolute raw count (baseline_i
% is never logged) -- and ONLY where the original value wasn't clipped to
% exactly 0 or 1 (clipping is not invertible; see the README).
SENSITIVITY = 30.0;

SURFACE_NAMES = {'flat', 'solid', 'hollow'};
[~, ~, own_cell_col] = muca_layout();

% ── Load ─────────────────────────────────────────────────────────────────
data = muca_load_three_surfaces(FLAT_CSV, SOLID_CSV, HOLLOW_CSV);

% ── Compute V0 / Press, one full 19-point picture PER indented point ────
% v0_by_ip.p3.flat  = 19x1 (median across P3's 10 iterations), etc.
% press_by_ip.p3.flat = 19x1 (median across P3's 10 iterations), etc.
v0_by_ip    = struct();
press_by_ip = struct();
deltaV_by_ip = struct();

% "Reverse" (partial-inverse) calculation: undoes the GAMMA power and the
% SENSITIVITY division from the sensor normalization formula, recovering
% (raw - baseline) in raw-sensor-count-equivalent units instead of the
% normalized [0,1] fraction. NOT valid where the underlying V hit the clip
% boundary (0 or 1 exactly) -- see the clipped-point warnings this script
% prints, and README_signal_normalization.md section 3.3/5.
rawdelta_v0_by_ip    = struct();
rawdelta_press_by_ip = struct();
rawdelta_swing_by_ip = struct();

% Intermediate stage: GAMMA undone (squared) but SENSITIVITY NOT yet
% re-applied -- this is V^2 = clip((raw-baseline)/SENSITIVITY, 0, 1), i.e.
% "without gamma," still expressed as a sensitivity-scaled ratio (stays in
% [0,1], same clip boundary as V itself). Multiplying this by SENSITIVITY
% is what turns it into rawdelta_* above ("without gamma AND without
% sensitivity" = raw-baseline).
ratio_v0_by_ip    = struct();
ratio_press_by_ip = struct();
ratio_swing_by_ip = struct();

% Range tables (built from the RAW per-iteration matrices, i.e. the true
% iteration-to-iteration spread -- not derived from the plotted medians):
% v0_range_by_ip.p3.min / .max are each 19x3 (point x surface), same for
% press_range_by_ip. Point j, surface s -> that (indented point, point,
% surface)'s [min max] across the 10 iterations.
v0_range_by_ip    = struct();
press_range_by_ip = struct();

for pi = 1:numel(POINT_IDS)
    ip = POINT_IDS(pi);
    ip_field = sprintf('p%d', ip);
    v0_by_ip.(ip_field)    = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
    press_by_ip.(ip_field) = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
    deltaV_by_ip.(ip_field) = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
    rawdelta_v0_by_ip.(ip_field)    = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
    rawdelta_press_by_ip.(ip_field) = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
    rawdelta_swing_by_ip.(ip_field) = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
    ratio_v0_by_ip.(ip_field)    = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
    ratio_press_by_ip.(ip_field) = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));
    ratio_swing_by_ip.(ip_field) = struct('flat', nan(19,1), 'solid', nan(19,1), 'hollow', nan(19,1));

    v0_min = nan(19, 3); v0_max = nan(19, 3);
    press_min = nan(19, 3); press_max = nan(19, 3);

    for s = 1:3
        sname = SURFACE_NAMES{s};
        seg = muca_extract_point_segments(data.(sname), ip, own_cell_col, MAX_ROUND, DEPTH_MM);

        % Per-iteration, per-point averages: (n_rounds_found x 19) each.
        % Row r = round r's mean across its raw release/hold samples, all
        % 19 columns kept (this point's own column AND cross-talk into
        % the other 18, all from THIS indented point's press cycle only).
        rounds_here = unique([seg.released.round_idx; seg.pressed.round_idx])';
        v0_iter    = nan(numel(rounds_here), 19);
        press_iter = nan(numel(rounds_here), 19);
        for ri = 1:numel(rounds_here)
            r = rounds_here(ri);
            v0_iter(ri, :)    = mean(seg.released.values(seg.released.round_idx == r, :), 1);
            press_iter(ri, :) = mean(seg.pressed.values(seg.pressed.round_idx == r, :), 1);
        end

        % Range across the 10 iterations, per point (column), THIS surface.
        v0_min(:, s)    = min(v0_iter, [], 1)';
        v0_max(:, s)    = max(v0_iter, [], 1)';
        press_min(:, s) = min(press_iter, [], 1)';
        press_max(:, s) = max(press_iter, [], 1)';

        % ---------------- YOUR CALCULATION GOES HERE ----------------
        % v0_iter / press_iter are (iterations x 19) -- median down each
        % column (across THIS indented point's iterations only) to get
        % the 19x1 that gets plotted. Swap median(...) for mean/prctile/
        % whatever you want, just keep it column-wise (dim=1).
        v0_by_ip.(ip_field).(sname)    = median(v0_iter, 1)';
        press_by_ip.(ip_field).(sname) = median(press_iter, 1)';

        % deltaV = Press - V0, taken PER ROUND (rows of v0_iter/press_iter
        % are round-aligned) before reducing across iterations, so each
        % round's own release/hold pair is what gets differenced.
        delta_iter = press_iter - v0_iter;
        deltaV_by_ip.(ip_field).(sname) = median(delta_iter, 1)';
        % --------------------------------------------------------------

        % ---------------- REVERSE (PARTIAL-INVERSE) CALCULATION --------
        % Undo GAMMA (square, since GAMMA=0.5) and SENSITIVITY (multiply
        % back) on the median-reduced V0/Press vectors above, recovering
        % (raw - baseline) in raw-count-equivalent units. Reverse V0 and
        % Press SEPARATELY, then subtract -- not the other way around --
        % since squaring a difference of two already-gamma'd values would
        % not undo anything meaningful.
        v0_vec    = v0_by_ip.(ip_field).(sname);
        press_vec = press_by_ip.(ip_field).(sname);
        ratio_v0_by_ip.(ip_field).(sname)    = v0_vec    .^ 2;
        ratio_press_by_ip.(ip_field).(sname) = press_vec .^ 2;
        ratio_swing_by_ip.(ip_field).(sname) = ratio_press_by_ip.(ip_field).(sname) ...
                                              - ratio_v0_by_ip.(ip_field).(sname);

        rawdelta_v0_by_ip.(ip_field).(sname)    = ratio_v0_by_ip.(ip_field).(sname)    * SENSITIVITY;
        rawdelta_press_by_ip.(ip_field).(sname) = ratio_press_by_ip.(ip_field).(sname) * SENSITIVITY;
        rawdelta_swing_by_ip.(ip_field).(sname) = rawdelta_press_by_ip.(ip_field).(sname) ...
                                                 - rawdelta_v0_by_ip.(ip_field).(sname);

        clipped_v0    = find(v0_vec    <= 1e-9 | v0_vec    >= 1 - 1e-9)';
        clipped_press = find(press_vec <= 1e-9 | press_vec >= 1 - 1e-9)';
        if ~isempty(clipped_v0) || ~isempty(clipped_press)
            fprintf('%-8s indented=P%02d : [warn] reverse V0/Press unreliable (clipped) at V0 points %s, Press points %s\n', ...
                sname, ip, mat2str(clipped_v0), mat2str(clipped_press));
        end
        % --------------------------------------------------------------

        fprintf('%-8s indented=P%02d : %d iterations -> v0 %dx19, press %dx19\n', ...
            sname, ip, numel(rounds_here), size(v0_iter, 1), size(press_iter, 1));
    end

    v0_range_by_ip.(ip_field)    = struct('min', v0_min,    'max', v0_max);
    press_range_by_ip.(ip_field) = struct('min', press_min, 'max', press_max);
end

% ── Range tables: console + CSV, one pair (V0, Press) per indented point ──
function print_and_save_range_table(metric_name, ip, mn, mx, RESULTS_DIR, SURFACE_NAMES)
% PRINT_AND_SAVE_RANGE_TABLE  Console-print + CSV-save a (point x surface)
% [min, max] range table. mn/mx are 19x3 (point x surface, surface order
% matching SURFACE_NAMES). One row per physical point 1-19, one min/max
% pair of columns per surface.
    fprintf('\n%s\n', repmat('=', 1, 78));
    fprintf('  %s RANGE (min-max across iterations) -- P%02d indented\n', upper(metric_name), ip);
    fprintf('%s\n', repmat('=', 1, 78));
    fprintf('  %5s |', 'Point');
    for s = 1:3
        fprintf(' %20s |', [upper(SURFACE_NAMES{s}(1)) SURFACE_NAMES{s}(2:end) ' (min - max)']);
    end
    fprintf('\n');
    for p = 1:19
        fprintf('  %5d |', p);
        for s = 1:3
            fprintf(' %8.4f - %8.4f |', mn(p, s), mx(p, s));
        end
        fprintf('\n');
    end

    csv_path = fullfile(RESULTS_DIR, sprintf('%s_range_p%d.csv', lower(metric_name), ip));
    fid = fopen(csv_path, 'w');
    header = 'point';
    for s = 1:3
        header = [header ',' SURFACE_NAMES{s} '_min,' SURFACE_NAMES{s} '_max']; %#ok<AGROW>
    end
    fprintf(fid, '%s\n', header);
    for p = 1:19
        row = sprintf('%d', p);
        for s = 1:3
            row = [row sprintf(',%.6f,%.6f', mn(p, s), mx(p, s))]; %#ok<AGROW>
        end
        fprintf(fid, '%s\n', row);
    end
    fclose(fid);
    fprintf('  saved: %s\n', csv_path);
end

for pi = 1:numel(POINT_IDS)
    ip = POINT_IDS(pi);
    ip_field = sprintf('p%d', ip);
    print_and_save_range_table('V0', ip, v0_range_by_ip.(ip_field).min, v0_range_by_ip.(ip_field).max, ...
        RESULTS_DIR, SURFACE_NAMES);
    print_and_save_range_table('Press', ip, press_range_by_ip.(ip_field).min, press_range_by_ip.(ip_field).max, ...
        RESULTS_DIR, SURFACE_NAMES);
end

% ── Shared color scale across V0, Press, AND deltaV -- so all 9 figures
%    (3 metrics x 3 indented points) are directly comparable: the same
%    color always means the same absolute value, on every figure ───────
all_shared_vals = [];
for pi = 1:numel(POINT_IDS)
    ip_field = sprintf('p%d', POINT_IDS(pi));
    for s = 1:3
        sname = SURFACE_NAMES{s};
        all_shared_vals = [all_shared_vals; v0_by_ip.(ip_field).(sname); ...
            press_by_ip.(ip_field).(sname); deltaV_by_ip.(ip_field).(sname)]; %#ok<AGROW>
    end
end
all_shared_vals = all_shared_vals(~isnan(all_shared_vals));

% Two variants to compare side by side:
%   datamin -- scale anchored to the true min (includes the small negative
%              dip from noise-floor cross-talk points where Press < V0)
%   zeromin -- scale anchored to exactly 0, so 0 always means "no signal"
%              and the negative dip clips to the bottom color instead
CLIM_VARIANTS = struct( ...
    'datamin', [min(all_shared_vals), max(all_shared_vals)], ...
    'zeromin', [0,                    max(all_shared_vals)] ...
);
variant_names = fieldnames(CLIM_VARIANTS);
for vi = 1:numel(variant_names)
    vname = variant_names{vi};
    fprintf('\nShared color scale (%s) across V0/Press/deltaV: [%.4f, %.4f]\n', ...
        vname, CLIM_VARIANTS.(vname)(1), CLIM_VARIANTS.(vname)(2));
end

% ── Plot: one hex comparison per indented point, per metric, per clim
%    variant (ip outlined in red -- these are cross-talk-style full-board
%    pictures) ────────────────────────────────────────────────────────
for vi = 1:numel(variant_names)
    vname = variant_names{vi};
    clim = CLIM_VARIANTS.(vname);

    for pi = 1:numel(POINT_IDS)
        ip = POINT_IDS(pi);
        ip_field = sprintf('p%d', ip);

        fig_v0 = muca_plot_hex_comparison(v0_by_ip.(ip_field), ...
            sprintf('V0 (release), median of 10 iterations -- P%02d indented [%s]', ip, vname), ...
            sprintf('Muca-Board -- V0 while P%02d was indented (median of 10 iterations, %s scale)', ip, vname), ip, clim);
        save_fig(fig_v0, fullfile(RESULTS_DIR, sprintf('v0_median_p%d_hexmap_%s', ip, vname)));

        fig_press = muca_plot_hex_comparison(press_by_ip.(ip_field), ...
            sprintf('Press (hold), median of 10 iterations -- P%02d indented [%s]', ip, vname), ...
            sprintf('Muca-Board -- Press while P%02d was indented (median of 10 iterations, %s scale)', ip, vname), ip, clim);
        save_fig(fig_press, fullfile(RESULTS_DIR, sprintf('press_median_p%d_hexmap_%s', ip, vname)));

        fig_dv = muca_plot_hex_comparison(deltaV_by_ip.(ip_field), ...
            sprintf('deltaV = Press - V0, median of 10 iterations -- P%02d indented [%s]', ip, vname), ...
            sprintf('Muca-Board -- deltaV (Press - V0) while P%02d was indented (median of 10 iterations, %s scale)', ip, vname), ip, clim);
        save_fig(fig_dv, fullfile(RESULTS_DIR, sprintf('deltaV_median_p%d_hexmap_%s', ip, vname)));
    end
end

% ── Shared color scale for the REVERSE (raw-delta-equivalent) maps -- kept
%    SEPARATE from SHARED_CLIM above: these are in raw-count-equivalent
%    units (~0-30), not the normalized [0,1] fraction, so they are not
%    on the same scale as V0/Press/deltaV and must not share a colorbar
%    with them ─────────────────────────────────────────────────────────
all_rawdelta_vals = [];
for pi = 1:numel(POINT_IDS)
    ip_field = sprintf('p%d', POINT_IDS(pi));
    for s = 1:3
        sname = SURFACE_NAMES{s};
        all_rawdelta_vals = [all_rawdelta_vals; rawdelta_v0_by_ip.(ip_field).(sname); ...
            rawdelta_press_by_ip.(ip_field).(sname); rawdelta_swing_by_ip.(ip_field).(sname)]; %#ok<AGROW>
    end
end
all_rawdelta_vals = all_rawdelta_vals(~isnan(all_rawdelta_vals));
RAWDELTA_CLIM = [min(all_rawdelta_vals), max(all_rawdelta_vals)];
fprintf('\nShared color scale across reverse-calc V0/Press/swing (raw-count-equivalent units): [%.4f, %.4f]\n', ...
    RAWDELTA_CLIM(1), RAWDELTA_CLIM(2));

% ── Plot: reverse (raw-delta-equivalent) hex maps, one per indented point,
%    per metric. Labeled explicitly as approximate/partial-inverse so
%    they're never mistaken for real raw sensor counts. ─────────────────
for pi = 1:numel(POINT_IDS)
    ip = POINT_IDS(pi);
    ip_field = sprintf('p%d', ip);

    fig_rv0 = muca_plot_hex_comparison(rawdelta_v0_by_ip.(ip_field), ...
        sprintf('V0 reverse-calc ~(raw-baseline), median of 10 iter -- P%02d indented', ip), ...
        sprintf('Muca-Board -- V0 reverse-calc (raw-count-equivalent, NOT true raw) while P%02d indented', ip), ip, RAWDELTA_CLIM);
    save_fig(fig_rv0, fullfile(RESULTS_DIR, sprintf('rawdelta_v0_median_p%d_hexmap', ip)));

    fig_rpress = muca_plot_hex_comparison(rawdelta_press_by_ip.(ip_field), ...
        sprintf('Press reverse-calc ~(raw-baseline), median of 10 iter -- P%02d indented', ip), ...
        sprintf('Muca-Board -- Press reverse-calc (raw-count-equivalent, NOT true raw) while P%02d indented', ip), ip, RAWDELTA_CLIM);
    save_fig(fig_rpress, fullfile(RESULTS_DIR, sprintf('rawdelta_press_median_p%d_hexmap', ip)));

    fig_rswing = muca_plot_hex_comparison(rawdelta_swing_by_ip.(ip_field), ...
        sprintf('Swing reverse-calc = Press_rev - V0_rev, median of 10 iter -- P%02d indented', ip), ...
        sprintf('Muca-Board -- swing reverse-calc (raw-count-equivalent) while P%02d indented', ip), ip, RAWDELTA_CLIM);
    save_fig(fig_rswing, fullfile(RESULTS_DIR, sprintf('rawdelta_swing_median_p%d_hexmap', ip)));
end

% ── Shared color scale for the "no gamma, sensitivity NOT yet re-applied"
%    ratio maps (V^2 -- stays in [0,1], but is NOT the same quantity as V0/
%    Press/deltaV, so kept on its own scale rather than reusing SHARED_CLIM
%    even though the numeric range happens to overlap) ─────────────────
all_ratio_vals = [];
for pi = 1:numel(POINT_IDS)
    ip_field = sprintf('p%d', POINT_IDS(pi));
    for s = 1:3
        sname = SURFACE_NAMES{s};
        all_ratio_vals = [all_ratio_vals; ratio_v0_by_ip.(ip_field).(sname); ...
            ratio_press_by_ip.(ip_field).(sname); ratio_swing_by_ip.(ip_field).(sname)]; %#ok<AGROW>
    end
end
all_ratio_vals = all_ratio_vals(~isnan(all_ratio_vals));
RATIO_CLIM = [min(all_ratio_vals), max(all_ratio_vals)];
fprintf('\nShared color scale across no-gamma ratio V0/Press/swing (still /SENSITIVITY, [0,1]-ish): [%.4f, %.4f]\n', ...
    RATIO_CLIM(1), RATIO_CLIM(2));

% ── Plot: "without gamma" hex maps (sensitivity still applied) ──────────
for pi = 1:numel(POINT_IDS)
    ip = POINT_IDS(pi);
    ip_field = sprintf('p%d', ip);

    fig_ratv0 = muca_plot_hex_comparison(ratio_v0_by_ip.(ip_field), ...
        sprintf('V0, gamma undone (still /SENSITIVITY), median of 10 iter -- P%02d indented', ip), ...
        sprintf('Muca-Board -- V0 without gamma, WITH sensitivity scaling, while P%02d indented', ip), ip, RATIO_CLIM);
    save_fig(fig_ratv0, fullfile(RESULTS_DIR, sprintf('nogamma_v0_median_p%d_hexmap', ip)));
    

    fig_ratpress = muca_plot_hex_comparison(ratio_press_by_ip.(ip_field), ...
        sprintf('Press, gamma undone (still /SENSITIVITY), median of 10 iter -- P%02d indented', ip), ...
        sprintf('Muca-Board -- Press without gamma, WITH sensitivity scaling, while P%02d indented', ip), ip, RATIO_CLIM);
    save_fig(fig_ratpress, fullfile(RESULTS_DIR, sprintf('nogamma_press_median_p%d_hexmap', ip)));

    fig_ratswing = muca_plot_hex_comparison(ratio_swing_by_ip.(ip_field), ...
        sprintf('Swing, gamma undone (still /SENSITIVITY), median of 10 iter -- P%02d indented', ip), ...
        sprintf('Muca-Board -- swing without gamma, WITH sensitivity scaling, while P%02d indented', ip), ip, RATIO_CLIM);
    save_fig(fig_ratswing, fullfile(RESULTS_DIR, sprintf('nogamma_swing_median_p%d_hexmap', ip)));
end

fprintf('\nFigures saved in: %s/\n', RESULTS_DIR);
