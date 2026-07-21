%% step2_ur_force_vs_time.m
%
% Step 2: for each recording confirmed in step1_loadcell_calibration.m,
% compare four forces over time during the loaded (measurement) window
% only -- baseline is left out here, step1 already used it for the fit:
%
%   F_expected     known force from the placed weight, as felt by the
%                  load cell (its hardware: holder/hook only)
%   F_expected_ur  the SAME placed weight, but as felt by the UR wrist --
%                  the UR also holds up the coupler, screws and the load
%                  cell's own body, so its hardware mass (and therefore
%                  its expected force) is not the same number
%   F_from_ai0     load cell voltage run through step1's fit
%   F_from_ur      UR wrist fz, sign-corrected to match the others
%
% Every plot uses the same time and force axis limits (the widest range
% found across all recordings), so magnitudes are directly comparable
% from one weight to the next.
%
% Run step1_loadcell_calibration.m first -- this reads its saved
% calibration and confirmed file list. Figures are shown one at a time
% (Enter advances, nothing is closed for you) and saved as .png, .fig
% and .svg in step2_plots/. At the end, a single panel figure with all
% 22 recordings (posz on top, negz on bottom, lightest to heaviest) is
% also shown and saved next to this script.

clear; clc; close all;

HARDWARE_G_LC.posz = 7;    % holder/hook only -- what the load cell itself feels
HARDWARE_G_LC.negz = 4;
HARDWARE_G_UR.posz = 50;   % + coupler (15g) + screws (21g) + load cell body (7g) --
HARDWARE_G_UR.negz = 47;   % what the UR wrist feels in this same rig
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
    cos_tilt = cos(deg2rad(meta.tilt_from_vertical_deg));

    r.fname         = fname;
    r.direction     = direction;
    r.weight_g      = info.weight_g;
    r.ts            = info.ts;
    r.t             = t;
    r.F_expected    = sign_d * ((HARDWARE_G_LC.(direction) + info.weight_g) / 1000) * G * cos_tilt;
    r.F_expected_ur = sign_d * ((HARDWARE_G_UR.(direction) + info.weight_g) / 1000) * G * cos_tilt;
    r.F_from_ai0    = slope * ai0 + offset;
    r.F_from_ur     = sign_d * abs(fz);

    if isempty(recs)
        recs = r;
    else
        recs(end + 1) = r; %#ok<AGROW>
    end

    t_max = max(t_max, t(end));
    f_min = min([f_min; r.F_expected; r.F_expected_ur; r.F_from_ai0; r.F_from_ur]);
    f_max = max([f_max; r.F_expected; r.F_expected_ur; r.F_from_ai0; r.F_from_ur]);
end

% posz first (lightest to heaviest), then negz -- same order the
% per-recording plots and the panel below both use. '0'/'1' prefix keeps
% posz ahead of negz (plain alphabetical order would put negz first).
sort_keys = cell(1, numel(recs));
for i = 1:numel(recs)
    if strcmp(recs(i).direction, 'posz')
        group = '0';
    else
        group = '1';
    end
    sort_keys{i} = sprintf('%s_%08.2f', group, recs(i).weight_g);
end
[~, order] = sort(sort_keys);
recs = recs(order);

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

    fprintf(['%2d/%d  %-5s %6.0fg  bias(ai0)=%+.4f N  bias(ur vs F_exp)=%+.4f N  ' ...
             'bias(ur vs F_exp_ur)=%+.4f N  -- %s\n'], ...
            i, numel(recs), r.direction, r.weight_g, ...
            mean(r.F_from_ai0) - r.F_expected, mean(r.F_from_ur) - r.F_expected, ...
            mean(r.F_from_ur) - r.F_expected_ur, r.fname);

    fig = figure('Color', 'w', 'Position', [100 100 760 480]);
    hold on
    plot([r.t(1) r.t(end)], [r.F_expected r.F_expected], 'k--', 'LineWidth', 1.5);
    plot([r.t(1) r.t(end)], [r.F_expected_ur r.F_expected_ur], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
    plot(r.t, r.F_from_ai0, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
    plot(r.t, r.F_from_ur, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.3);
    set(gca, 'FontName', 'Helvetica', 'FontSize', 10, 'Box', 'off');
    xlim(x_limits);
    ylim(y_limits);
    legend({'F_{expected}', 'F_{expected,UR}', 'F_{from ai0}', 'F_{from UR fz}'}, 'Location', 'best');
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

%% ---- Panel: all recordings in one grid, posz on top row, negz on bottom ----

posz_recs = recs(strcmp({recs.direction}, 'posz'));
negz_recs = recs(strcmp({recs.direction}, 'negz'));
n_cols = max(numel(posz_recs), numel(negz_recs));
rows = {posz_recs, negz_recs};
row_labels = {'posz (push)', 'negz (pull)'};

panel_fig = figure('Color', 'w', 'Position', [50 50 1900 700]);
for row = 1:2
    row_recs = rows{row};
    for col = 1:numel(row_recs)
        r = row_recs(col);
        ax = subplot(2, n_cols, (row - 1) * n_cols + col);
        hold(ax, 'on');
        plot(ax, [r.t(1) r.t(end)], [r.F_expected r.F_expected], 'k--', 'LineWidth', 1);
        plot(ax, [r.t(1) r.t(end)], [r.F_expected_ur r.F_expected_ur], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
        plot(ax, r.t, r.F_from_ai0, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1);
        plot(ax, r.t, r.F_from_ur, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1);
        set(ax, 'FontName', 'Helvetica', 'FontSize', 7, 'Box', 'off');
        xlim(ax, x_limits);
        ylim(ax, y_limits);
        title(ax, sprintf('%.0fg', r.weight_g), 'FontSize', 8);
        if col == 1
            ylabel(ax, sprintf('%s\nforce (N)', row_labels{row}));
        else
            set(ax, 'YTickLabel', []);
        end
        if row == 2
            xlabel(ax, 'time (s)');
        else
            set(ax, 'XTickLabel', []);
        end
    end
end

% Panel title and legend go on their own invisible full-width axes
% (top and bottom) instead of sgtitle/per-subplot legends -- sgtitle
% isn't available on GNU Octave, which this script is also tested on.
title_ax = axes(panel_fig, 'Position', [0 0.96 1 0.04], 'Visible', 'off');
text(title_ax, 0.5, 0.5, ...
     'Step 2 -- all confirmed recordings, posz (top) and negz (bottom), 10g to 1156g', ...
     'HorizontalAlignment', 'center', 'FontName', 'Helvetica', 'FontSize', 12, 'FontWeight', 'bold');

lg_ax = axes(panel_fig, 'Position', [0 0 1 0.04], 'Visible', 'off');
hold(lg_ax, 'on');
h1 = plot(lg_ax, NaN, NaN, 'k--', 'LineWidth', 1.5);
h2 = plot(lg_ax, NaN, NaN, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
h3 = plot(lg_ax, NaN, NaN, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
h4 = plot(lg_ax, NaN, NaN, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5);
legend(lg_ax, [h1 h2 h3 h4], {'F_{expected}', 'F_{expected,UR}', 'F_{from ai0}', 'F_{from UR fz}'}, ...
       'Orientation', 'horizontal', 'Location', 'south', 'Box', 'off');

panel_base = fullfile(HERE, 'step2_panel_overview');
print(panel_fig, [panel_base '.png'], '-dpng', '-r150');
print(panel_fig, [panel_base '.svg'], '-dsvg');
savefig(panel_fig, [panel_base '.fig']);

fprintf('\nSaved panel overview -> %s.png / .fig / .svg\n', panel_base);
