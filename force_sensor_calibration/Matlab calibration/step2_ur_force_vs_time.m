%% step2_ur_force_vs_time.m
%
% Step 2: for each recording confirmed in step1_loadcell_calibration.m,
% compare three forces over time during the loaded (measurement) window
% only -- baseline is left out here, step1 already used it for the fit:
%
%   F_expected  known force from the placed weight (constant)
%   F_from_ai0  load cell voltage run through step1's fit
%   F_from_ur   UR wrist fz, sign-corrected to match the other two
%
% Every plot uses the same time and force axis limits (the widest range
% found across all recordings), so magnitudes are directly comparable
% from one weight to the next.
%
% Run step1_loadcell_calibration.m first -- this reads its saved
% calibration and confirmed file list. Figures are shown one at a time
% (Enter advances, nothing is closed for you) and saved as .png, .fig
% and .svg in step2_plots/.

clear; clc; close all;

HARDWARE_G.posz = 7;      % holder
HARDWARE_G.negz = 4;      % hook
AI0_SIGN.posz = -1;
AI0_SIGN.negz = +1;
G = 9.80665;

HERE = fileparts(mfilename('fullpath'));
CALIB_DIR = fileparts(HERE);
LOG_DIR = fullfile(CALIB_DIR, 'logs');

step1_path = fullfile(HERE, 'step1_loadcell_calibration.json');
if ~exist(step1_path, 'file')
    error('Run step1_loadcell_calibration.m first (need step1_loadcell_calibration.json).');
end
step1 = jsondecode(fileread(step1_path));
slope = step1.slope_n_per_v;
offset = step1.offset_n;

confirmed_files = step1.confirmed_files;
if ischar(confirmed_files)          % jsondecode gives a plain char row for a 1-element list
    confirmed_files = {confirmed_files};
end

fprintf('F_from_ai0 = %.4f * ai0 + (%.4f)\n', slope, offset);
fprintf('%d confirmed recordings\n\n', numel(confirmed_files));

%% ---- Pass 1: load every recording's measurement window, track the widest range ----

recs = struct([]);
t_max = 0;
f_min = Inf;
f_max = -Inf;

for i = 1:numel(confirmed_files)
    fname = confirmed_files{i};
    csv_path = fullfile(LOG_DIR, fname);
    meta_path = strrep(csv_path, '.csv', '_meta.json');

    if ~exist(csv_path, 'file')
        warning('%s not found, skipping', fname);
        continue
    end
    info = parse_recording_name(fname);
    if isempty(info)
        warning('%s does not parse, skipping', fname);
        continue
    end
    meta = jsondecode(fileread(meta_path));
    if ~isempty(info.direction)
        direction = info.direction;
    else
        direction = meta.axis;   % v2 negz files drop the direction token
    end

    [ts, loaded, ai0, fz] = read_measurement_columns(csv_path);
    is_loaded = loaded == 1;
    if ~any(is_loaded)
        warning('%s has no loaded==1 rows, skipping', fname);
        continue
    end

    t   = ts(is_loaded) - ts(find(is_loaded, 1, 'first'));
    ai0 = ai0(is_loaded);
    fz  = fz(is_loaded);

    sign_d   = AI0_SIGN.(direction);
    hw_g     = HARDWARE_G.(direction);
    cos_tilt = cos(deg2rad(meta.tilt_from_vertical_deg));

    r.fname      = fname;
    r.direction  = direction;
    r.weight_g   = info.weight_g;
    r.ts         = info.ts;
    r.t          = t;
    r.F_expected = sign_d * ((hw_g + info.weight_g) / 1000) * G * cos_tilt;
    r.F_from_ai0 = slope * ai0 + offset;
    r.F_from_ur  = sign_d * abs(fz);

    if isempty(recs)
        recs = r;
    else
        recs(end + 1) = r; %#ok<AGROW>
    end

    t_max = max(t_max, t(end));
    f_min = min([f_min; r.F_expected; r.F_from_ai0; r.F_from_ur]);
    f_max = max([f_max; r.F_expected; r.F_from_ai0; r.F_from_ur]);
end

f_margin = 0.05 * (f_max - f_min);
y_limits = [f_min - f_margin, f_max + f_margin];
x_limits = [0, t_max];

fprintf('shared axes: time [0, %.2f] s, force [%.2f, %.2f] N\n\n', t_max, y_limits(1), y_limits(2));

%% ---- Pass 2: one figure per recording, same axes every time ----

OUT_DIR = fullfile(HERE, 'step2_plots');
if ~exist(OUT_DIR, 'dir')
    mkdir(OUT_DIR);
end

for i = 1:numel(recs)
    r = recs(i);

    fprintf('%2d/%d  %-5s %6.0fg  bias(ai0)=%+.4f N  bias(ur)=%+.4f N  -- %s\n', ...
            i, numel(recs), r.direction, r.weight_g, ...
            mean(r.F_from_ai0) - r.F_expected, mean(r.F_from_ur) - r.F_expected, r.fname);

    fig = figure('Color', 'w', 'Position', [100 100 760 480]);
    hold on
    plot([r.t(1) r.t(end)], [r.F_expected r.F_expected], 'k--', 'LineWidth', 1.5);
    plot(r.t, r.F_from_ai0, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
    plot(r.t, r.F_from_ur, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.3);
    set(gca, 'FontName', 'Helvetica', 'FontSize', 10, 'Box', 'off');
    xlim(x_limits);
    ylim(y_limits);
    legend({'F_{expected}', 'F_{from ai0}', 'F_{from UR fz}'}, 'Location', 'best');
    xlabel('time since start of measurement (s)');
    ylabel('force (N)');
    title(sprintf('%s   %.0f g   %s', r.direction, r.weight_g, strrep(r.ts, '_', ' ')));
    grid off

    [~, base_name] = fileparts(r.fname);
    out_base = fullfile(OUT_DIR, [base_name '_step2']);
    print(fig, [out_base '.png'], '-dpng', '-r150');
    print(fig, [out_base '.svg'], '-dsvg');
    savefig(fig, [out_base '.fig']);

    if i < numel(recs)
        input('Press Enter for the next figure... ', 's');
    end
end
