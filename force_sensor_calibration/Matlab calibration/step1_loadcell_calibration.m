%% step1_loadcell_calibration.m
%
% STEP 1 of the force-sensor calibration, on its own, written for a
% human operator. It answers one question:
%
%     "How many Newtons is one Volt from the FUTEK load cell?"
%
% What happens when you run it:
%
%   A. It searches ../logs/ for the futek_direct recordings belonging
%      to the batch selected in DATASET_VERSION below ('v2' today).
%   B. It shows you a list of EXACTLY which files it intends to use
%      and then STOPS. Nothing is fitted until you type y and press
%      Enter. If the list is not what you expected, type anything else
%      and the script quits without touching anything.
%   C. It fits the straight line   F = slope * ai0 + offset   through
%      all confirmed recordings and reports the load-cell sensitivity
%      in N/V and mV/N.
%   D. It saves a figure (PNG) and the fitted numbers (JSON) next to
%      this script, including the list of files you confirmed, so
%      later steps of the linearization can use the same selection.
%
% This is a trimmed-down copy of ../matlab/a1_fit_lc_ur_calibration.m
% (Step 1 only) and follows the same two conventions:
%
%   * The no-load "baseline" is NOT zero force: the plastic holder
%     (posz) or the metal hook (negz) is already resting on the load
%     cell while the baseline is recorded. So every recording gives
%     TWO known points -- (baseline = hardware only) and
%     (loaded = hardware + test weight) -- not one.
%   * Pushing (posz) and pulling (negz) get opposite signs (AI0_SIGN)
%     so both directions land on one single line.
%
% HOW TO RUN: open this file in MATLAB and press Run (F5), or cd into
% this folder and type  step1_loadcell_calibration  in the command
% window. No toolboxes required.

clear; clc; close all;

%% ================= SETTINGS (edit here if needed) =================

% Which collection batch to use. Files carry the tag in their name
% (e.g. fzcal_futek_direct_100g_v2_20260715_160629.csv); files with no
% tag are the original batch, 'v1'. When a new batch is collected,
% change this one line -- nothing else in the script needs editing.
DATASET_VERSION = 'v2';

% Hardware resting on the load cell during the recordings (grams),
% confirmed by whoever ran the collection.
HARDWARE_G.posz = 7;      % plastic holder
HARDWARE_G.negz = 4;      % metal hook

% Sign convention -- a hardware fact, do not change: pushing (posz)
% drives the bridge voltage DOWN, pulling (negz) drives it UP.
AI0_SIGN.posz = -1;
AI0_SIGN.negz = +1;

G = 9.80665;              % standard gravity, m/s^2

%% ================= A. Find the recordings =========================

HERE = fileparts(mfilename('fullpath'));   % .../Matlab calibration
CALIB_DIR = fileparts(HERE);               % .../force_sensor_calibration
LOG_DIR = fullfile(CALIB_DIR, 'logs');

% Each filename is decoded by parse_recording_name.m (a small helper
% next to this script). It handles both filename forms:
%   fzcal_futek_direct_posz_100g_v2_20260715_153459.csv   (direction in name)
%   fzcal_futek_direct_100g_v2_20260715_160629.csv        (no posz/negz in the
%       name -- the direction then comes from the recording's own
%       _meta.json file, "axis" field)
% It deliberately uses plain string splitting, NOT regular expressions:
% MATLAB's regexp engine silently fails to match optional tagged groups,
% which twice made this discovery find zero recordings.

files = dir(fullfile(LOG_DIR, 'fzcal_futek_direct_*.csv'));
sessions_cell = {};
for k = 1:numel(files)
    info = parse_recording_name(files(k).name);
    if isempty(info)
        continue                         % not a calibration recording
    end
    if ~strcmp(info.version, DATASET_VERSION)
        continue                         % not the batch we want -- skip
    end

    s.csv_path  = fullfile(files(k).folder, files(k).name);
    s.meta_path = strrep(s.csv_path, '.csv', '_meta.json');
    s.weight_g  = info.weight_g;
    s.ts        = info.ts;
    if ~isempty(info.direction)
        s.direction = info.direction;
    else
        meta = jsondecode(fileread(s.meta_path));
        s.direction = meta.axis;
    end
    sessions_cell{end + 1} = s; %#ok<AGROW>
end

if isempty(sessions_cell)
    error('No %s futek_direct recordings found in %s -- check DATASET_VERSION.', ...
          DATASET_VERSION, LOG_DIR);
end
sessions = [sessions_cell{:}];

% If the same direction+weight was recorded more than once inside this
% batch, keep only the most recent attempt.
keys = cell(1, numel(sessions));
for i = 1:numel(sessions)
    keys{i} = sprintf('%s_%d', sessions(i).direction, round(sessions(i).weight_g));
end
[~, order] = sort({sessions.ts});          % oldest -> newest
sessions = sessions(order);
keys = keys(order);
[~, keep_idx] = unique(keys, 'last');      % last occurrence = newest
sessions = sessions(sort(keep_idx));

% Sort the list for display: posz first, then negz, lightest to heaviest.
sort_keys = cell(1, numel(sessions));
for i = 1:numel(sessions)
    sort_keys{i} = sprintf('%s_%07d', sessions(i).direction, round(sessions(i).weight_g));
end
[~, order] = sort(sort_keys);
sessions = sessions(order);

%% ================= B. Show the plan, ask for confirmation =========

n = numel(sessions);
fprintf('\nThese %d recordings (batch %s) will be used for Step 1:\n\n', ...
        n, DATASET_VERSION);
fprintf('   #   direction   weight    recorded              file\n');
fprintf('  ---  ---------   -------   -------------------   ----------------------------------------------\n');
for i = 1:n
    s = sessions(i);
    ts = s.ts;                                    % '20260715_153459'
    recorded = [ts(1:4) '-' ts(5:6) '-' ts(7:8) ' ' ts(10:11) ':' ts(12:13) ':' ts(14:15)];
    [~, base_name, ext] = fileparts(s.csv_path);
    fprintf('  %3d  %-9s   %5.0f g   %s   %s%s\n', ...
            i, s.direction, s.weight_g, recorded, base_name, ext);
end
n_posz = sum(strcmp({sessions.direction}, 'posz'));
fprintf('\n  posz (push): %d recordings, negz (pull): %d recordings\n', ...
        n_posz, n - n_posz);

fprintf('\n');   % (input() prints its prompt literally, so the blank line goes here)
reply = input('Proceed with these datasets? Type y to continue, anything else to stop: ', 's');
if ~strcmpi(strtrim(reply), 'y')
    fprintf(['\nStopped -- nothing was fitted and nothing was saved.\n' ...
             'To change the selection: edit DATASET_VERSION at the top of this\n' ...
             'script, or add/remove recordings in\n  %s\n'], LOG_DIR);
    return
end

%% ================= C. Step 1 fit: Volts -> Newtons ================

% Every recording contributes 2 points: baseline (hardware only) and
% loaded (hardware + test weight). Force is signed by direction.
ai0_pts   = zeros(2 * n, 1);
F_pts     = zeros(2 * n, 1);
is_loaded = false(2 * n, 1);
pt_dir    = cell(2 * n, 1);

for i = 1:n
    s = sessions(i);
    meta = jsondecode(fileread(s.meta_path));
    [loaded_col, ai0_col] = read_loaded_and_ai0(s.csv_path);

    ai0_baseline = mean(ai0_col(loaded_col == 0));
    ai0_loaded   = mean(ai0_col(loaded_col == 1));

    sign_d   = AI0_SIGN.(s.direction);
    hw_g     = HARDWARE_G.(s.direction);
    cos_tilt = cos(deg2rad(meta.tilt_from_vertical_deg));

    F_baseline = sign_d * (hw_g / 1000) * G * cos_tilt;
    F_loaded   = sign_d * ((hw_g + s.weight_g) / 1000) * G * cos_tilt;

    ai0_pts(2*i - 1) = ai0_baseline;  F_pts(2*i - 1) = F_baseline;
    ai0_pts(2*i)     = ai0_loaded;    F_pts(2*i)     = F_loaded;
    is_loaded(2*i)   = true;
    pt_dir{2*i - 1}  = s.direction;   pt_dir{2*i}    = s.direction;
end

p = polyfit(ai0_pts, F_pts, 1);
slope  = p(1);                             % N per Volt
offset = p(2);                             % N
F_pred = slope * ai0_pts + offset;
resid  = F_pts - F_pred;
r2     = 1 - sum(resid.^2) / sum((F_pts - mean(F_pts)).^2);
rmse   = sqrt(mean(resid.^2));

fprintf('\n==================== STEP 1 RESULT ====================\n');
fprintf('F [N] = %.4f * ai0 [V] + (%.4f)\n', slope, offset);
fprintf('Load-cell sensitivity : %.4f N/V   (%.3f mV/N)\n', slope, 1000 / slope);
fprintf('Fit quality           : R^2 = %.5f   RMSE = %.4f N   (n = %d points)\n', ...
        r2, rmse, numel(ai0_pts));
fprintf('=======================================================\n');

%% ================= D. Figure + saved coefficients =================

fig = figure('Color', 'w', 'Position', [100 100 720 520]);
ax = axes('Parent', fig);
hold(ax, 'on');
set(ax, 'FontName', 'Helvetica', 'FontSize', 10, 'Box', 'off');

% Only the loaded points are drawn (the baseline points still feed the
% fit -- they are just visually redundant, all sitting near no-load).
posz_pt = strcmp(pt_dir, 'posz') & is_loaded;
negz_pt = strcmp(pt_dir, 'negz') & is_loaded;
plot(ax, ai0_pts(posz_pt), F_pts(posz_pt), 'o', 'MarkerSize', 8, ...
     'MarkerFaceColor', [0.85 0.33 0.10], 'MarkerEdgeColor', 'none', ...
     'LineStyle', 'none', 'DisplayName', 'posz (push)');
plot(ax, ai0_pts(negz_pt), F_pts(negz_pt), 's', 'MarkerSize', 8, ...
     'MarkerFaceColor', [0.00 0.45 0.74], 'MarkerEdgeColor', 'none', ...
     'LineStyle', 'none', 'DisplayName', 'negz (pull)');

x_line = linspace(min(ai0_pts), max(ai0_pts), 100);
plot(ax, x_line, slope * x_line + offset, 'k-', 'LineWidth', 1.5, ...
     'DisplayName', sprintf('fit: F = %.3f*ai0 + %.3f  (R^2 = %.4f)', slope, offset, r2));

xlabel(ax, 'load-cell bridge voltage ai0 (V)');
ylabel(ax, 'known force on the load cell (N), signed');
title(ax, sprintf('Step 1 -- load-cell voltage vs force (batch %s, loaded points only)', ...
      DATASET_VERSION), 'FontName', 'Helvetica');
legend(ax, 'Location', 'northwest');
grid(ax, 'off');

png_path = fullfile(HERE, 'step1_loadcell_calibration.png');
print(fig, png_path, '-dpng', '-r150');    % 'print' works on both old MATLAB and Octave
fprintf('\nSaved figure -> %s\n', png_path);

% Save the numbers AND the confirmed file list, so the later
% linearization steps can reuse exactly this dataset selection.
confirmed_files = cell(n, 1);
for i = 1:n
    [~, base_name, ext] = fileparts(sessions(i).csv_path);
    confirmed_files{i} = [base_name ext];
end

out.dataset_version      = DATASET_VERSION;
out.date                 = datestr(now, 'yyyy-mm-dd');
out.slope_n_per_v        = slope;
out.offset_n             = offset;
out.sensitivity_mv_per_n = 1000 / slope;
out.r_squared            = r2;
out.rmse_n               = rmse;
out.n_points             = numel(ai0_pts);
out.confirmed_files      = confirmed_files;

json_path = fullfile(HERE, 'step1_loadcell_calibration.json');
try
    json_text = jsonencode(out, 'PrettyPrint', true);   % needs R2021a+
catch
    json_text = jsonencode(out);                        % older MATLAB: compact JSON
end
fid = fopen(json_path, 'w');
fprintf(fid, '%s\n', json_text);
fclose(fid);
fprintf('Saved coefficients + confirmed file list -> %s\n', json_path);

% (The small CSV-reading helper lives in read_loaded_and_ai0.m, next to
% this script.)
