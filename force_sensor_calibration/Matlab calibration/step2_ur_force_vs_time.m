%% step2_ur_force_vs_time.m
%
% STEP 2 of the force-sensor calibration, on its own, written for a
% human operator. It answers one question:
%
%     "During the measurement itself, how does the UR robot's own
%      force reading compare to the force we expect, and to the force
%      computed from the load cell's voltage?"
%
% What happens when you run it:
%
%   A. It reads step1_loadcell_calibration.json, saved next to this
%      script by step1_loadcell_calibration.m -- the load-cell fit
%      (F_from_ai0 = slope*ai0 + offset) AND the exact list of files the
%      operator confirmed there. Step 2 reuses that same confirmed list
%      instead of asking again, so the two steps always agree on which
%      recordings are in use. Run Step 1 first.
%   B. For every confirmed recording, it keeps ONLY the rows where
%      loaded == 1 -- the measurement part, once the test weight is
%      actually on the sensor. The no-load baseline segment at the
%      start of each recording is left out of these plots on purpose
%      (Step 1 still uses it for its fit; this script is specifically
%      about the loaded measurement window).
%   C. For that window it plots THREE things against time:
%        F_expected  -- the known force from the placed weight (flat --
%                        it does not change during the window)
%        F_from_ai0  -- the load cell's voltage run through Step 1's
%                        fit, sample by sample
%        F_from_ur   -- the UR wrist sensor's own fz reading, sample by
%                        sample, sign-corrected (push vs pull) to the
%                        same convention as the other two
%      and prints, per recording, how far the two measured curves sit
%      from the expected value on average.
%   D. It saves one PNG per confirmed recording into step2_plots/, next
%      to this script (figures are not popped open on screen -- with
%      ~20 recordings that would be ~20 windows -- open the PNGs to
%      look at them).
%
% HOW TO RUN: run step1_loadcell_calibration.m first and confirm a
% dataset. Then open this file in MATLAB and press Run (F5), or cd into
% this folder and type  step2_ur_force_vs_time  in the command window.
% No toolboxes required.

clear; clc; close all;

%% ================= Constants (must match Step 1) ==================

HARDWARE_G.posz = 7;      % plastic holder
HARDWARE_G.negz = 4;      % metal hook
AI0_SIGN.posz = -1;
AI0_SIGN.negz = +1;
G = 9.80665;               % standard gravity, m/s^2

%% ================= A. Load Step 1's calibration + file list =======

HERE = fileparts(mfilename('fullpath'));
CALIB_DIR = fileparts(HERE);
LOG_DIR = fullfile(CALIB_DIR, 'logs');

step1_path = fullfile(HERE, 'step1_loadcell_calibration.json');
if ~exist(step1_path, 'file')
    error(['step1_loadcell_calibration.json not found next to this script.\n' ...
           'Run step1_loadcell_calibration.m first and confirm a dataset -- ' ...
           'Step 2 reuses its calibration numbers and its confirmed file list.']);
end
step1 = jsondecode(fileread(step1_path));
slope  = step1.slope_n_per_v;
offset = step1.offset_n;

confirmed_files = step1.confirmed_files;
if ischar(confirmed_files)          % jsondecode returns a plain char row
    confirmed_files = {confirmed_files};   % when there was only 1 file
end

fprintf('\nUsing Step 1 calibration (batch %s, confirmed %s):\n', ...
        step1.dataset_version, step1.date);
fprintf('  F_from_ai0 = %.4f * ai0 + (%.4f)\n', slope, offset);
fprintf('  %d confirmed recording(s) to plot\n', numel(confirmed_files));

%% ================= B-D. Per-recording measurement window ===========

OUT_DIR = fullfile(HERE, 'step2_plots');
if ~exist(OUT_DIR, 'dir')
    mkdir(OUT_DIR);
end

fprintf('\n   #  direction   weight    bias(ai0)     bias(UR fz)    file\n');
fprintf('  ---  ---------   -------   -----------   ------------   ------------------------------\n');

n_ok = 0;
for i = 1:numel(confirmed_files)
    fname = confirmed_files{i};
    csv_path  = fullfile(LOG_DIR, fname);
    meta_path = strrep(csv_path, '.csv', '_meta.json');

    if ~exist(csv_path, 'file')
        warning('Skipping %s -- file no longer in logs/.', fname);
        continue
    end

    info = parse_recording_name(fname);
    if isempty(info)
        warning('Skipping %s -- name no longer parses.', fname);
        continue
    end
    meta = jsondecode(fileread(meta_path));
    if ~isempty(info.direction)
        direction = info.direction;
    else
        direction = meta.axis;
    end

    [ts_col, loaded_col, ai0_col, fz_col] = read_measurement_columns(csv_path);
    is_loaded = loaded_col == 1;
    if ~any(is_loaded)
        warning('Skipping %s -- no loaded==1 rows.', fname);
        continue
    end

    t   = ts_col(is_loaded) - ts_col(find(is_loaded, 1, 'first'));  % s since window start
    ai0 = ai0_col(is_loaded);
    fz  = fz_col(is_loaded);

    sign_d   = AI0_SIGN.(direction);
    hw_g     = HARDWARE_G.(direction);
    cos_tilt = cos(deg2rad(meta.tilt_from_vertical_deg));

    F_expected = sign_d * ((hw_g + info.weight_g) / 1000) * G * cos_tilt;
    F_from_ai0 = slope * ai0 + offset;
    F_from_ur  = sign_d * abs(fz);   % same push/pull convention as F_expected/F_from_ai0

    bias_ai0 = mean(F_from_ai0) - F_expected;
    bias_ur  = mean(F_from_ur) - F_expected;

    fig = figure('Color', 'w', 'Position', [100 100 760 480], 'Visible', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    set(ax, 'FontName', 'Helvetica', 'FontSize', 10, 'Box', 'off');

    plot(ax, [t(1) t(end)], [F_expected F_expected], 'k--', 'LineWidth', 1.5);
    plot(ax, t, F_from_ai0, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
    plot(ax, t, F_from_ur,  '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.3);

    legend(ax, {'F_{expected}', 'F_{from ai0}', 'F_{from UR fz}'}, ...
           'Location', 'best');
    xlabel(ax, 'time since start of measurement (s)');
    ylabel(ax, 'force (N)');
    title(ax, sprintf('%s   %.0f g   %s', direction, info.weight_g, ...
          strrep(info.ts, '_', ' ')), 'FontName', 'Helvetica');
    grid(ax, 'off');

    [~, base_name] = fileparts(fname);
    png_path = fullfile(OUT_DIR, [base_name '_step2.png']);
    print(fig, png_path, '-dpng', '-r150');
    close(fig);

    n_ok = n_ok + 1;
    fprintf('  %3d  %-9s   %5.0f g   %+9.4f N   %+10.4f N   %s\n', ...
            n_ok, direction, info.weight_g, bias_ai0, bias_ur, fname);
end

fprintf('\nbias = mean(measured) - F_expected over the measurement window (positive = reads high)\n');
fprintf('Saved %d plot(s) -> %s\n', n_ok, OUT_DIR);
