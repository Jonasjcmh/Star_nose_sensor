%% step3_futek_vs_linearization_barplot.m
%
% STEP 3 of the force-sensor calibration. For each confirmed load
% (every direction/weight combination from step1_loadcell_calibration.m),
% draws one bar pair:
%
%   F_futek   the known reference force the FUTEK load cell feels for
%             that load (hardware + placed weight, signed by direction) --
%             RED bar
%   F_lin     that SAME recording's own ai0 voltage run through step1's
%             linearization  F = slope*ai0 + offset  -- BLUE bar
%
% Both bars keep their natural sign: pushing (posz) and pulling (negz)
% land on opposite sides of zero (see AI0_SIGN below), nothing is
% converted to absolute value. This lets a mis-fit linearization show up
% as a bar pair that leans the wrong way, not just a size mismatch.
%
% Run step1_loadcell_calibration.m first -- this reads its saved
% calibration and confirmed file list, exactly like step2 does.
% Saves step3_futek_vs_linearization_barplot.png / .svg / .fig next to
% this script.

clear; clc; close all;

HARDWARE_G_LC.posz = 7;    % holder/hook only -- what the load cell itself feels
HARDWARE_G_LC.negz = 4;
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

fprintf('F_lin = %.4f * ai0 + (%.4f)\n', slope, offset);
fprintf('%d confirmed recordings\n\n', numel(confirmed_files));

%% ---- Load every recording, compute the two bar values -------------------

recs = struct([]);
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

    [~, loaded, ai0, ~] = read_measurement_columns(csv_path);
    is_loaded = loaded == 1;
    if ~any(is_loaded)
        warning('%s has no loaded==1 rows, skipping', fname);
        continue
    end
    ai0_mean = mean(ai0(is_loaded));

    sign_d   = AI0_SIGN.(direction);
    cos_tilt = cos(deg2rad(meta.tilt_from_vertical_deg));

    r.fname      = fname;
    r.direction  = direction;
    r.weight_g   = info.weight_g;
    r.F_futek    = sign_d * ((HARDWARE_G_LC.(direction) + info.weight_g) / 1000) * G * cos_tilt;
    r.F_lin      = slope * ai0_mean + offset;

    if isempty(recs)
        recs = r;
    else
        recs(end + 1) = r; %#ok<AGROW>
    end
end

% posz first (lightest to heaviest), then negz -- same ordering step2 uses.
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

n = numel(recs);
F_futek = [recs.F_futek]';
F_lin   = [recs.F_lin]';
err_n   = F_lin - F_futek;

labels = cell(n, 1);
for i = 1:n
    labels{i} = sprintf('%s %.0fg', recs(i).direction, recs(i).weight_g);
    fprintf('%2d/%d  %-5s %6.0fg  F_futek=%+7.3f N  F_lin=%+7.3f N  err=%+.4f N\n', ...
            i, n, recs(i).direction, recs(i).weight_g, recs(i).F_futek, recs(i).F_lin, err_n(i));
end

%% ---- Bar plot: FUTEK reference (red) vs linearization calc (blue) --------

RED  = [0.80 0.10 0.10];
BLUE = [0.00 0.45 0.74];

fig = figure('Color', 'w', 'Position', [50 50 1500 650]);
ax = axes('Parent', fig);
hold(ax, 'on');

hb = bar(ax, [F_futek, F_lin], 'grouped', 'BarWidth', 0.85);
set(hb(1), 'FaceColor', RED,  'EdgeColor', 'none');
set(hb(2), 'FaceColor', BLUE, 'EdgeColor', 'none');

plot(ax, [0.5, n + 0.5], [0 0], 'k-', 'LineWidth', 0.8);

set(ax, 'FontName', 'Helvetica', 'FontSize', 9, 'Box', 'off');
set(ax, 'XTick', 1:n, 'XTickLabel', labels, 'XTickLabelRotation', 60);
xlim(ax, [0.5, n + 0.5]);
xlabel(ax, 'load (direction, placed weight)');
ylabel(ax, 'force (N), signed -- push (posz) and pull (negz) on opposite sides of zero');
title(ax, 'Step 3 -- FUTEK reference vs linearized calculation, per load', ...
      'FontName', 'Helvetica');
legend(ax, {'F_{futek} (known reference force)', ...
            sprintf('F_{lin} = %.4f {\\cdot} ai0 + (%.4f)', slope, offset)}, ...
       'Location', 'best');
grid(ax, 'off');

png_path = fullfile(HERE, 'step3_futek_vs_linearization_barplot.png');
svg_path = fullfile(HERE, 'step3_futek_vs_linearization_barplot.svg');
fig_path = fullfile(HERE, 'step3_futek_vs_linearization_barplot.fig');
print(fig, png_path, '-dpng', '-r150');
print(fig, svg_path, '-dsvg');
savefig(fig, fig_path);

fprintf('\nSaved bar plot -> %s\n', png_path);
fprintf('Saved bar plot -> %s\n', svg_path);
fprintf('Saved bar plot -> %s\n', fig_path);
