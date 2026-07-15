%% plot_lc_ur_force_vs_time.m
%
% MATLAB port of plot_lc_ur_force_vs_time.py.
%
% For the fzcal_futek_direct_* sessions (load cell installed alongside the
% UR robot), converts the load cell's raw ai0 voltage to force using the
% fitted linear transformation and plots it together with the UR robot's
% own force (fz) on the SAME force axis, vs time. Also produces the
% Force-vs-Voltage linearization plot the transformation comes from.
%
% This script is self-contained, like the other MATLAB scripts in this
% folder -- it repeats the same discovery / de-duplication / fit logic.
%
% Voltage -> force linear transformation
% ---------------------------------------
% Uses EVERY individual raw sample from the loaded window of every session
% (200 samples/session here), not just the session mean, plus one baseline
% point per session (the mean -- baseline is a known, non-zero load, the
% hardware mass alone, not a zero reference; see a1_fit_lc_ur_calibration.m).
% Fit: F_signed = m_v * ai0 + c_v.
%
% UR force display convention (this plot only)
% -----------------------------------------------
% The UR's raw fz doesn't follow the load cell's signed convention (posz
% negative, negz positive) -- both directions mostly read negative on fz
% directly. For THIS plot only, fz is re-signed to match the load cell's
% convention for direct visual comparison: F_ur_display = ai0_sign*|fz|,
% i.e. negative for posz and positive for negz, same as F_lc. Display
% transform only -- does not change fz itself or the Step 2/3 compensation
% fit elsewhere.
%
% Panel ordering (Force vs time)
% --------------------------------
% Panels run continuously from -200 g to +200 g using the load cell's own
% sign convention as the ordering key (posz -> negative, negz -> positive,
% same as F_signed) -- NOT grouped by direction. So the sequence is
% posz 200/100/50/20/10/5 g, then negz 5/10/20/50/100/200 g, laid out
% left-to-right, top-to-bottom.
%
% UR compensation coefficients (new)
% ------------------------------------
% A separate fit, pooling EVERY raw sample (both baseline and loaded, both
% directions) with F_lc (from the voltage<->force fit) as the reference
% "real" value and the UR's raw, actually-signed fz as the value to
% compensate: F_lc = comp_a * fz_raw + comp_b. Reported alongside its
% per-direction breakdown, since posz and negz do not agree.
%
% Outputs
% -------
%   plots/lc_ur_force_vs_time_matlab.png  -- 2 rows x 6 columns, ordered
%     -200 g to +200 g, F_lc(t) and F_ur_display(t) overlaid on the same
%     force axis, loaded window only, same y-scale across all panels.
%   plots/lc_linearization_matlab.png     -- the Force vs Voltage fit,
%     showing EVERY raw loaded sample (not just the mean), the baseline
%     anchor per session, the fit line, and per-weight/direction
%     annotations with the point count and the hardware-compensated
%     weight (nominal + hardware).
%   plots/ur_compensation_linearization_matlab.png -- F_lc (from the load
%     cell) vs fz, every sample, both directions pooled -200..+200 g: raw
%     fz (left) and sign-corrected fz (right), with fit coefficients.
%   plots/ur_vs_trueweight_linearization_matlab.png -- same idea, but
%     against F_true (the KNOWN weight + hardware mass, bypassing the
%     load cell entirely) instead of F_lc -- the fundamental ground-truth
%     check.
%
% All fit coefficients found are printed in one consolidated summary at
% the end of the run.
%
% Run this file directly (F5, or "run plot_lc_ur_force_vs_time" from the
% force_sensor_calibration/matlab folder). No toolboxes required.

clear; clc; close all;

%% ---- Paths & constants ----

HERE = fileparts(mfilename('fullpath'));
CALIB_DIR = fileparts(HERE);
LOG_DIR = fullfile(CALIB_DIR, 'logs');
OUT_DIR = fullfile(CALIB_DIR, 'plots');
if ~exist(OUT_DIR, 'dir')
    mkdir(OUT_DIR);
end

G = 9.80665;
STANDARD_WEIGHTS_G = [5 10 20 50 100 200];

%% ---- Discover + de-duplicate the load-cell-installed sessions ----

entries = discover_entries(LOG_DIR, 'futek_direct', STANDARD_WEIGHTS_G);
entries = dedupe_latest(entries);

sessions_cell = cell(1, numel(entries));
for i = 1:numel(entries)
    sessions_cell{i} = load_session(entries(i), G);
end
sessions = sort_sessions([sessions_cell{:}]);

%% ---- Build the fit dataset: baseline mean anchor + EVERY raw loaded
%     sample, per session -- and keep the per-(direction, weight) raw
%     loaded arrays around for the plot/annotations. ----

fit_ai0 = [];
fit_f = [];
cluster_cell = cell(1, numel(sessions));   % one element per (direction, weight)

for i = 1:numel(sessions)
    s = sessions(i);
    entry = find_entry(entries, s.direction, s.weight_g);
    [t, is_loaded, fz, ai0] = load_raw_series(entry.csv_path); %#ok<ASGLU>
    ai0_loaded_raw = ai0(is_loaded);

    fit_ai0 = [fit_ai0; s.ai0_base_mean; ai0_loaded_raw]; %#ok<AGROW>
    fit_f = [fit_f; s.F_signed_base; repmat(s.F_signed, numel(ai0_loaded_raw), 1)]; %#ok<AGROW>

    hardware_g = extra_hardware_futek_direct(s.direction);
    c.direction = s.direction;
    c.weight_g = s.weight_g;
    c.ai0_loaded_raw = ai0_loaded_raw;
    c.ai0_base_mean = s.ai0_base_mean;
    c.F_signed = s.F_signed;
    c.F_signed_base = s.F_signed_base;
    c.n_loaded = numel(ai0_loaded_raw);
    c.compensated_g = s.weight_g + hardware_g;
    cluster_cell{i} = c;
end
cluster = [cluster_cell{:}];

[m_v, c_v, r2_v, rmse_v] = linfit(fit_ai0, fit_f);
fprintf('Linearization uses %d points (%d baseline means + %d raw loaded samples)\n', ...
        numel(fit_ai0), numel(sessions), numel(fit_ai0) - numel(sessions));
fprintf('F = %.4f*ai0 + (%.5f)  (R^2=%.5f, RMSE=%.4f N)\n', m_v, c_v, r2_v, rmse_v);

%% ---- Pass 1: load every panel, convert ai0->F_lc, re-sign fz to match
%     the load cell's convention, track global y-range. Also pull each
%     panel's EXPECTED (ground-truth) levels: F_signed for the load cell
%     (hardware = holder/hook only, 7g/4g) and F_signed_ur for the UR
%     sensor (hardware = coupler+screws+LC body+holder/hook, 50g/47g) --
%     genuinely different ground truths for the two instruments in this
%     SAME rig (see a1_fit_lc_ur_calibration.m). ----

% Signed ordering: -200 g ... +200 g, posz negative / negz positive, same
% key as F_signed. NOT grouped by direction.
ordered_dirs = [repmat({'posz'}, 1, 6), repmat({'negz'}, 1, 6)];
ordered_weights = [fliplr(STANDARD_WEIGHTS_G), STANDARD_WEIGHTS_G];
n_panels = numel(ordered_dirs);
ncols = numel(STANDARD_WEIGHTS_G);

panel_data = cell(1, n_panels);
y_lo = Inf; y_hi = -Inf;

for i = 1:n_panels
    direction = ordered_dirs{i};
    weight_g = ordered_weights(i);
    sign_d = ai0_sign(direction);
    entry = find_entry(entries, direction, weight_g);
    if isempty(entry)
        continue
    end

    [t, is_loaded, fz, ai0] = load_raw_series(entry.csv_path);
    fz_display = sign_d * abs(fz);
    fz_base_mean = mean(fz_display(~is_loaded));
    ai0_base_mean = mean(ai0(~is_loaded));

    load_start = t(find(is_loaded, 1, 'first'));
    t_load = t(is_loaded) - load_start;
    fz_load = fz_display(is_loaded);
    f_lc_load = m_v * ai0(is_loaded) + c_v;

    s_ground = find_session(sessions, direction, weight_g);
    f_lc_expected = s_ground.F_signed;
    f_ur_expected = s_ground.F_signed_ur;

    panel_data{i} = struct('t', t_load, 'f_ur', fz_load, 'f_lc', f_lc_load, ...
        'direction', direction, 'weight_g', weight_g, 'signed_weight', sign_d * weight_g, ...
        'dFz', mean(fz_load) - fz_base_mean, ...
        'dFlc', mean(f_lc_load) - (m_v * ai0_base_mean + c_v), ...
        'f_lc_expected', f_lc_expected, 'f_ur_expected', f_ur_expected);

    y_lo = min([y_lo; fz_load; f_lc_load; f_lc_expected; f_ur_expected]);
    y_hi = max([y_hi; fz_load; f_lc_load; f_lc_expected; f_ur_expected]);
end

margin = 0.05 * (y_hi - y_lo);
y_range = [y_lo - margin, y_hi + margin];

%% ---- Pass 2: plot the grid, 2 rows x 6 columns filled in signed order ----

fig = figure('Color', 'w', 'Position', [50 50 3600 1050]);

for i = 1:n_panels
    ax = subplot(2, ncols, i);

    d = panel_data{i};
    if isempty(d)
        set(ax, 'Visible', 'off');
        continue
    end

    hold(ax, 'on');
    plot(ax, d.t, d.f_lc, 'Color', [0.10 0.43 0.71], 'LineWidth', 1.2);
    plot(ax, d.t, d.f_ur, 'Color', [0.84 0.15 0.16], 'LineWidth', 1.2);
    yline(ax, d.f_lc_expected, 'Color', [0.50 0.72 0.88], 'LineStyle', ':', 'LineWidth', 1.3);
    yline(ax, d.f_ur_expected, 'Color', [0.91 0.60 0.60], 'LineStyle', ':', 'LineWidth', 1.3);
    ylim(ax, y_range);

    is_row2 = i > ncols;
    is_col1 = mod(i - 1, ncols) == 0;

    title(ax, sprintf('%+.0f g  (%s)\ndFlc=%+.3f N, dFz=%+.3f N', d.signed_weight, d.direction, ...
          d.dFlc, d.dFz), 'FontSize', 9);
    if is_row2
        xlabel(ax, 'time since load start (s)', 'FontSize', 8);
    end
    if is_col1
        ylabel(ax, 'Force (N)', 'FontSize', 8);
    end
    if i == 1
        legend(ax, {'F_{lc} (load cell)', 'F_{ur} (UR robot, LC sign convention)', ...
                    'expected F_{lc}', 'expected F_{ur}'}, ...
               'FontSize', 6, 'Location', 'southeast');
    end
    grid(ax, 'off');
end

sgtitle(fig, ['Load-cell force vs UR robot force, vs time, loaded window only' newline ...
              'futek\_direct sessions -- ordered -200 g to +200 g (posz negative, negz positive), same y-scale' newline ...
              'F_{ur} re-signed to match LC convention: negative=posz, positive=negz (display only)   |   ' ...
              'dotted lines: expected F_{lc}/F_{ur} from known weight + hardware mass']);

out_path = fullfile(OUT_DIR, 'lc_ur_force_vs_time_matlab.png');
print(fig, out_path, '-dpng', '-r150');
fprintf('Saved -> %s\n', out_path);

%% ---- Linearization plot: every raw loaded sample + baseline anchors ----

fig2 = figure('Color', 'w', 'Position', [100 100 1300 950]);
ax2 = axes(fig2);
hold(ax2, 'on');

weight_order = STANDARD_WEIGHTS_G;
for k = 1:numel(cluster)
    c = cluster(k);
    color = weight_color(c.weight_g);
    if strcmp(c.direction, 'posz')
        marker = 'o';
    else
        marker = 's';
    end

    % every raw loaded sample -- a faint cloud, not a single mean point
    scatter(ax2, c.ai0_loaded_raw, repmat(c.F_signed, size(c.ai0_loaded_raw)), 14, color, ...
            'filled', 'MarkerFaceAlpha', 0.25, 'Marker', marker);

    % baseline anchor (session mean) -- open marker
    plot(ax2, c.ai0_base_mean, c.F_signed_base, marker, 'Color', color, ...
         'MarkerFaceColor', 'none', 'MarkerSize', 8, 'LineWidth', 1.5);

    % cluster mean (loaded) vs its estimated point, with a residual connector
    ai0_mean = mean(c.ai0_loaded_raw);
    f_est = m_v * ai0_mean + c_v;
    plot(ax2, [ai0_mean ai0_mean], [c.F_signed f_est], ':', 'Color', color, 'LineWidth', 1);
    plot(ax2, ai0_mean, c.F_signed, marker, 'Color', 'k', 'MarkerFaceColor', color, ...
         'MarkerSize', 9, 'LineWidth', 1.2);
    plot(ax2, ai0_mean, f_est, 'x', 'Color', color, 'MarkerSize', 9, 'LineWidth', 2);

    % annotation: point count + hardware-compensated weight. Offset scales
    % with weight index so the small-weight cluster (close together near
    % F=0) doesn't overlap; direction sets which side.
    idx = find(weight_order == c.weight_g, 1);
    if strcmp(c.direction, 'negz')
        y_off_data = (0.05 + (idx - 1) * 0.05) * (y_hi - y_lo) / 4;
        ha = 'left';
    else
        y_off_data = -(0.05 + (idx - 1) * 0.05) * (y_hi - y_lo) / 4;
        ha = 'right';
    end
    label = sprintf('%dg->%dg (n=%d)', round(c.weight_g), round(c.compensated_g), c.n_loaded);
    text(ax2, ai0_mean, c.F_signed + y_off_data, label, 'Color', color, 'FontSize', 6.5, ...
         'HorizontalAlignment', ha);
    plot(ax2, [ai0_mean ai0_mean], [c.F_signed, c.F_signed + y_off_data * 0.85], '-', ...
         'Color', color, 'LineWidth', 0.5);
end

margin2 = 0.05 * (max(fit_ai0) - min(fit_ai0));
x_range = linspace(min(fit_ai0) - margin2, max(fit_ai0) + margin2, 200);
plot(ax2, x_range, m_v * x_range + c_v, 'k-', 'LineWidth', 2);
yline(ax2, 0, 'Color', [0.5 0.5 0.5]);
xlabel(ax2, 'ai0, absolute (V)');
ylabel(ax2, 'F_{signed} (N)');
title(ax2, sprintf(['FUTEK load cell linearization -- Force vs Voltage\n' ...
      'F = %.4f*ai0 + (%.4f)  (R^2=%.4f, n=%d points)\n' ...
      'annotations: nominal weight -> hardware-compensated weight (n loaded samples)'], ...
      m_v, c_v, r2_v, numel(fit_ai0)));
grid(ax2, 'off');

% manual legend
weight_values = [5 10 20 50 100 200];
legend_handles = gobjects(1, numel(weight_values) + 4);
for k = 1:numel(weight_values)
    legend_handles(k) = plot(ax2, NaN, NaN, 'o', 'Color', weight_color(weight_values(k)), ...
        'MarkerFaceColor', weight_color(weight_values(k)), 'MarkerSize', 8, ...
        'DisplayName', sprintf('%d g', weight_values(k)));
end
legend_handles(end - 3) = plot(ax2, NaN, NaN, 'o', 'Color', [0.2 0.2 0.2], ...
    'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 5, 'DisplayName', 'raw loaded samples (cloud)');
legend_handles(end - 2) = plot(ax2, NaN, NaN, 'o', 'Color', 'k', ...
    'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 8, 'DisplayName', 'loaded mean (black edge)');
legend_handles(end - 1) = plot(ax2, NaN, NaN, 'o', 'Color', [0.2 0.2 0.2], ...
    'MarkerFaceColor', 'none', 'MarkerSize', 8, 'DisplayName', 'baseline mean (open)');
legend_handles(end) = plot(ax2, NaN, NaN, 'x', 'Color', [0.2 0.2 0.2], 'MarkerSize', 8, ...
    'LineWidth', 1.8, 'DisplayName', 'estimated (fit prediction)');
legend(ax2, legend_handles, 'FontSize', 7.5, 'NumColumns', 2, 'Location', 'northwest');

out_path2 = fullfile(OUT_DIR, 'lc_linearization_matlab.png');
print(fig2, out_path2, '-dpng', '-r150');
fprintf('Saved -> %s\n', out_path2);

%% ---- UR compensation linearization: F_lc (real value) vs fz (UR
%     sensor), EVERY sample (baseline + loaded), both directions pooled,
%     ordered -200..+200 g. Two versions:
%       raw:  fz as actually recorded (real signed reading) -- the honest
%             diagnostic; posz and negz disagree in sign behavior, so one
%             pooled fit is a poor compromise (left panel).
%       sign-corrected: fz_signed = ai0_sign(direction)*|fz|, the same
%             display convention used in the force-vs-time plot, so both
%             F_lc and fz share one reference and trace a single line
%             (right panel) -- but this requires knowing which direction
%             (sign) the applied load is in beforehand, since that's what
%             picks the sign to apply to |fz|. ----

comp_fz = [];
comp_fz_signed = [];
comp_flc = [];
comp_dir = {};
comp_weight = [];

for i = 1:n_panels
    direction = ordered_dirs{i};
    weight_g = ordered_weights(i);
    entry = find_entry(entries, direction, weight_g);
    if isempty(entry)
        continue
    end
    [t, is_loaded, fz, ai0] = load_raw_series(entry.csv_path); %#ok<ASGLU>
    f_lc_all = m_v * ai0 + c_v;
    fz_signed = ai0_sign(direction) * abs(fz);
    n = numel(fz);
    comp_fz = [comp_fz; fz]; %#ok<AGROW>
    comp_fz_signed = [comp_fz_signed; fz_signed]; %#ok<AGROW>
    comp_flc = [comp_flc; f_lc_all]; %#ok<AGROW>
    comp_dir = [comp_dir, repmat({direction}, 1, n)]; %#ok<AGROW>
    comp_weight = [comp_weight; repmat(weight_g, n, 1)]; %#ok<AGROW>
end

[comp_a, comp_b, comp_r2, comp_rmse] = linfit(comp_fz, comp_flc);
fprintf('\nUR compensation, raw fz (pooled, %d samples, -200..+200 g): F_lc = %.4f*fz_raw + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        numel(comp_fz), comp_a, comp_b, comp_r2, comp_rmse);
fprintf('  -> apply as: fz_corrected = %.4f * fz_raw + (%.5f)\n', comp_a, comp_b);

is_posz_c = strcmp(comp_dir, 'posz')';
is_negz_c = strcmp(comp_dir, 'negz')';
[posz_a, posz_b, posz_r2, posz_rmse] = linfit(comp_fz(is_posz_c), comp_flc(is_posz_c));
[negz_a, negz_b, negz_r2, negz_rmse] = linfit(comp_fz(is_negz_c), comp_flc(is_negz_c));
fprintf('  posz: F_lc = %.4f*fz_raw + (%.5f)   R^2 = %.5f   RMSE = %.4f N   (n=%d)\n', ...
        posz_a, posz_b, posz_r2, posz_rmse, sum(is_posz_c));
fprintf('  negz: F_lc = %.4f*fz_raw + (%.5f)   R^2 = %.5f   RMSE = %.4f N   (n=%d)\n', ...
        negz_a, negz_b, negz_r2, negz_rmse, sum(is_negz_c));

[comp_a2, comp_b2, comp_r2_2, comp_rmse2] = linfit(comp_fz_signed, comp_flc);
fprintf('\nUR compensation, sign-corrected fz (pooled, %d samples): F_lc = %.4f*fz_signed + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        numel(comp_fz_signed), comp_a2, comp_b2, comp_r2_2, comp_rmse2);
fprintf('  -> apply as: fz_corrected = %.4f * (ai0_sign(direction)*|fz_raw|) + (%.5f)\n', comp_a2, comp_b2);

fig3 = figure('Color', 'w', 'Position', [100 100 2100 900]);

weight_values = [5 10 20 50 100 200];
weight_handles3 = gobjects(1, numel(weight_values));

% --- Panel A: raw fz (honest diagnostic) ---
ax3a = subplot(1, 2, 1);
hold(ax3a, 'on');
for i = 1:n_panels
    direction = ordered_dirs{i};
    weight_g = ordered_weights(i);
    mask = strcmp(comp_dir, direction)' & (comp_weight == weight_g);
    color = weight_color(weight_g);
    if strcmp(direction, 'posz')
        marker = 'o';
    else
        marker = 's';
    end
    scatter(ax3a, comp_fz(mask), comp_flc(mask), 10, color, 'filled', ...
            'MarkerFaceAlpha', 0.2, 'Marker', marker);
end
margin3 = 0.05 * (max(comp_fz) - min(comp_fz));
x_range3 = linspace(min(comp_fz) - margin3, max(comp_fz) + margin3, 200);
h_pooled = plot(ax3a, x_range3, comp_a * x_range3 + comp_b, 'k-', 'LineWidth', 2, ...
    'DisplayName', sprintf('pooled: F=%.3f*fz+%.3f (R^2=%.4f)', comp_a, comp_b, comp_r2));
h_posz = plot(ax3a, x_range3, posz_a * x_range3 + posz_b, ':', 'Color', [0.12 0.47 0.71], 'LineWidth', 1.8, ...
    'DisplayName', sprintf('posz-only: F=%.3f*fz+%.3f (R^2=%.4f)', posz_a, posz_b, posz_r2));
h_negz = plot(ax3a, x_range3, negz_a * x_range3 + negz_b, ':', 'Color', [0.84 0.15 0.16], 'LineWidth', 1.8, ...
    'DisplayName', sprintf('negz-only: F=%.3f*fz+%.3f (R^2=%.4f)', negz_a, negz_b, negz_r2));
yline(ax3a, 0, 'Color', [0.5 0.5 0.5]);
xline(ax3a, 0, 'Color', [0.5 0.5 0.5]);
xlabel(ax3a, 'fz_{raw} (N) -- UR sensor, real signed reading');
ylabel(ax3a, 'F_{lc} (N) -- load cell, real value');
title(ax3a, sprintf(['Raw fz (honest diagnostic)' newline ...
      'posz/negz disagree -- one pooled fit is a poor compromise']));
grid(ax3a, 'off');
for k = 1:numel(weight_values)
    weight_handles3(k) = plot(ax3a, NaN, NaN, 'o', 'Color', weight_color(weight_values(k)), ...
        'MarkerFaceColor', weight_color(weight_values(k)), 'MarkerSize', 8, ...
        'DisplayName', sprintf('%d g', weight_values(k)));
end
legend(ax3a, [weight_handles3, h_pooled, h_posz, h_negz], 'FontSize', 7, 'NumColumns', 2, 'Location', 'northwest');

% --- Panel B: sign-corrected fz (same LC reference) ---
ax3b = subplot(1, 2, 2);
hold(ax3b, 'on');
for i = 1:n_panels
    direction = ordered_dirs{i};
    weight_g = ordered_weights(i);
    mask = strcmp(comp_dir, direction)' & (comp_weight == weight_g);
    color = weight_color(weight_g);
    if strcmp(direction, 'posz')
        marker = 'o';
    else
        marker = 's';
    end
    scatter(ax3b, comp_fz_signed(mask), comp_flc(mask), 10, color, 'filled', ...
            'MarkerFaceAlpha', 0.2, 'Marker', marker);
end
margin3b = 0.05 * (max(comp_fz_signed) - min(comp_fz_signed));
x_range3b = linspace(min(comp_fz_signed) - margin3b, max(comp_fz_signed) + margin3b, 200);
h_signed = plot(ax3b, x_range3b, comp_a2 * x_range3b + comp_b2, 'k-', 'LineWidth', 2, ...
    'DisplayName', sprintf('F_lc=%.3f*fz_signed+%.3f (R^2=%.4f)', comp_a2, comp_b2, comp_r2_2));
yline(ax3b, 0, 'Color', [0.5 0.5 0.5]);
xline(ax3b, 0, 'Color', [0.5 0.5 0.5]);
xlabel(ax3b, 'fz_{signed} = ai0\_sign(direction)*|fz_{raw}| (N)');
ylabel(ax3b, 'F_{lc} (N) -- load cell, real value');
title(ax3b, sprintf(['Sign-corrected fz (same LC reference)' newline ...
      'single linear relationship, but needs direction known beforehand']));
grid(ax3b, 'off');
legend(ax3b, [weight_handles3, h_signed], 'FontSize', 7, 'NumColumns', 2, 'Location', 'northwest');

sgtitle(fig3, sprintf('UR compensation linearization -- F_{lc} (real) vs fz (UR sensor), every sample, -200..+200 g pooled (n=%d)', ...
        numel(comp_fz)));

out_path3 = fullfile(OUT_DIR, 'ur_compensation_linearization_matlab.png');
print(fig3, out_path3, '-dpng', '-r150');
fprintf('Saved -> %s\n', out_path3);

%% ---- UR vs KNOWN weight (ground truth) linearization: same idea as the
%     LC compensation above, but skips the load cell entirely. Uses
%     F_signed_ur / F_signed_ur_base -- NOT F_signed/F_signed_base, which
%     are the LOAD CELL's own ground truth (hardware = holder/hook only,
%     7g/4g). The UR sensor holds up the load cell's own body too, so its
%     ground truth needs the larger hardware total (50g/47g) -- see
%     extra_hardware_futek_direct_ur. ----

truth_fz = [];
truth_fz_signed = [];
truth_ftrue = [];
truth_dir = {};
truth_weight = [];

for i = 1:numel(sessions)
    s = sessions(i);
    entry = find_entry(entries, s.direction, s.weight_g);
    [t, is_loaded, fz, ai0] = load_raw_series(entry.csv_path); %#ok<ASGLU>
    fz_signed = ai0_sign(s.direction) * abs(fz);
    n_base = sum(~is_loaded);
    n_load = sum(is_loaded);

    truth_fz = [truth_fz; fz(~is_loaded); fz(is_loaded)]; %#ok<AGROW>
    truth_fz_signed = [truth_fz_signed; fz_signed(~is_loaded); fz_signed(is_loaded)]; %#ok<AGROW>
    truth_ftrue = [truth_ftrue; repmat(s.F_signed_ur_base, n_base, 1); repmat(s.F_signed_ur, n_load, 1)]; %#ok<AGROW>
    truth_dir = [truth_dir, repmat({s.direction}, 1, n_base + n_load)]; %#ok<AGROW>
    truth_weight = [truth_weight; repmat(s.weight_g, n_base + n_load, 1)]; %#ok<AGROW>
end

[truth_a, truth_b, truth_r2, truth_rmse] = linfit(truth_fz, truth_ftrue);
fprintf('\nUR vs known weight, raw fz (pooled, %d samples): F_true = %.4f*fz_raw + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        numel(truth_fz), truth_a, truth_b, truth_r2, truth_rmse);

is_posz_t = strcmp(truth_dir, 'posz')';
is_negz_t = strcmp(truth_dir, 'negz')';
[truth_posz_a, truth_posz_b, truth_posz_r2, truth_posz_rmse] = linfit(truth_fz(is_posz_t), truth_ftrue(is_posz_t));
[truth_negz_a, truth_negz_b, truth_negz_r2, truth_negz_rmse] = linfit(truth_fz(is_negz_t), truth_ftrue(is_negz_t));
fprintf('  posz: F_true = %.4f*fz_raw + (%.5f)   R^2 = %.5f   RMSE = %.4f N   (n=%d)\n', ...
        truth_posz_a, truth_posz_b, truth_posz_r2, truth_posz_rmse, sum(is_posz_t));
fprintf('  negz: F_true = %.4f*fz_raw + (%.5f)   R^2 = %.5f   RMSE = %.4f N   (n=%d)\n', ...
        truth_negz_a, truth_negz_b, truth_negz_r2, truth_negz_rmse, sum(is_negz_t));

[truth_a2, truth_b2, truth_r2_2, truth_rmse2] = linfit(truth_fz_signed, truth_ftrue);
fprintf('\nUR vs known weight, sign-corrected fz (pooled, %d samples): F_true = %.4f*fz_signed + (%.5f)   R^2 = %.5f   RMSE = %.4f N\n', ...
        numel(truth_fz_signed), truth_a2, truth_b2, truth_r2_2, truth_rmse2);
fprintf('  -> apply as: fz_corrected = %.4f * (ai0_sign(direction)*|fz_raw|) + (%.5f)\n', truth_a2, truth_b2);

fig4 = figure('Color', 'w', 'Position', [100 100 2100 900]);

% --- Panel A: raw fz vs known weight ---
ax4a = subplot(1, 2, 1);
hold(ax4a, 'on');
for i = 1:n_panels
    direction = ordered_dirs{i};
    weight_g = ordered_weights(i);
    mask = strcmp(truth_dir, direction)' & (truth_weight == weight_g);
    color = weight_color(weight_g);
    if strcmp(direction, 'posz')
        marker = 'o';
    else
        marker = 's';
    end
    scatter(ax4a, truth_fz(mask), truth_ftrue(mask), 10, color, 'filled', ...
            'MarkerFaceAlpha', 0.2, 'Marker', marker);
end
margin4 = 0.05 * (max(truth_fz) - min(truth_fz));
x_range4 = linspace(min(truth_fz) - margin4, max(truth_fz) + margin4, 200);
h_pooled4 = plot(ax4a, x_range4, truth_a * x_range4 + truth_b, 'k-', 'LineWidth', 2, ...
    'DisplayName', sprintf('pooled: F_true=%.3f*fz+%.3f (R^2=%.4f)', truth_a, truth_b, truth_r2));
h_posz4 = plot(ax4a, x_range4, truth_posz_a * x_range4 + truth_posz_b, ':', 'Color', [0.12 0.47 0.71], 'LineWidth', 1.8, ...
    'DisplayName', sprintf('posz-only: F_true=%.3f*fz+%.3f (R^2=%.4f)', truth_posz_a, truth_posz_b, truth_posz_r2));
h_negz4 = plot(ax4a, x_range4, truth_negz_a * x_range4 + truth_negz_b, ':', 'Color', [0.84 0.15 0.16], 'LineWidth', 1.8, ...
    'DisplayName', sprintf('negz-only: F_true=%.3f*fz+%.3f (R^2=%.4f)', truth_negz_a, truth_negz_b, truth_negz_r2));
yline(ax4a, 0, 'Color', [0.5 0.5 0.5]);
xline(ax4a, 0, 'Color', [0.5 0.5 0.5]);
xlabel(ax4a, 'fz_{raw} (N) -- UR sensor, real signed reading');
ylabel(ax4a, 'F_{true} (N) -- known weight + hardware, signed');
title(ax4a, sprintf(['Raw fz vs KNOWN weight (ground truth)' newline ...
      'no load cell involved -- posz/negz still disagree']));
grid(ax4a, 'off');
weight_handles4 = gobjects(1, numel(weight_values));
for k = 1:numel(weight_values)
    weight_handles4(k) = plot(ax4a, NaN, NaN, 'o', 'Color', weight_color(weight_values(k)), ...
        'MarkerFaceColor', weight_color(weight_values(k)), 'MarkerSize', 8, ...
        'DisplayName', sprintf('%d g', weight_values(k)));
end
legend(ax4a, [weight_handles4, h_pooled4, h_posz4, h_negz4], 'FontSize', 7, 'NumColumns', 2, 'Location', 'northwest');

% --- Panel B: sign-corrected fz vs known weight ---
ax4b = subplot(1, 2, 2);
hold(ax4b, 'on');
for i = 1:n_panels
    direction = ordered_dirs{i};
    weight_g = ordered_weights(i);
    mask = strcmp(truth_dir, direction)' & (truth_weight == weight_g);
    color = weight_color(weight_g);
    if strcmp(direction, 'posz')
        marker = 'o';
    else
        marker = 's';
    end
    scatter(ax4b, truth_fz_signed(mask), truth_ftrue(mask), 10, color, 'filled', ...
            'MarkerFaceAlpha', 0.2, 'Marker', marker);
end
margin4b = 0.05 * (max(truth_fz_signed) - min(truth_fz_signed));
x_range4b = linspace(min(truth_fz_signed) - margin4b, max(truth_fz_signed) + margin4b, 200);
h_signed4 = plot(ax4b, x_range4b, truth_a2 * x_range4b + truth_b2, 'k-', 'LineWidth', 2, ...
    'DisplayName', sprintf('F_true=%.3f*fz_signed+%.3f (R^2=%.4f)', truth_a2, truth_b2, truth_r2_2));
yline(ax4b, 0, 'Color', [0.5 0.5 0.5]);
xline(ax4b, 0, 'Color', [0.5 0.5 0.5]);
xlabel(ax4b, 'fz_{signed} = ai0\_sign(direction)*|fz_{raw}| (N)');
ylabel(ax4b, 'F_{true} (N) -- known weight + hardware, signed');
title(ax4b, sprintf(['Sign-corrected fz vs KNOWN weight (ground truth)' newline ...
      'final compensation curve, no load cell needed']));
grid(ax4b, 'off');
legend(ax4b, [weight_handles4, h_signed4], 'FontSize', 7, 'NumColumns', 2, 'Location', 'northwest');

sgtitle(fig4, sprintf('UR sensor vs KNOWN weight (load + hardware), every sample, -200..+200 g pooled (n=%d)', ...
        numel(truth_fz)));

out_path4 = fullfile(OUT_DIR, 'ur_vs_trueweight_linearization_matlab.png');
print(fig4, out_path4, '-dpng', '-r150');
fprintf('Saved -> %s\n', out_path4);

%% ---- Consolidated coefficients summary, every curve fit in this run ----

fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('COEFFICIENTS SUMMARY -- all curves fit in this run\n');
fprintf('%s\n', repmat('=', 1, 78));
fprintf('1. LC linearization (ai0 -> F_signed), n=%d:\n', numel(fit_ai0));
fprintf('     F_signed = %.4f * ai0 + (%.5f)   R^2=%.5f  RMSE=%.4f N\n', m_v, c_v, r2_v, rmse_v);
fprintf('\n2. UR compensation vs F_lc (load cell), raw fz, n=%d:\n', numel(comp_fz));
fprintf('     pooled : F_lc = %.4f*fz + (%.5f)   R^2=%.5f\n', comp_a, comp_b, comp_r2);
fprintf('     posz   : F_lc = %.4f*fz + (%.5f)   R^2=%.5f\n', posz_a, posz_b, posz_r2);
fprintf('     negz   : F_lc = %.4f*fz + (%.5f)   R^2=%.5f\n', negz_a, negz_b, negz_r2);
fprintf('\n3. UR compensation vs F_lc (load cell), sign-corrected fz, n=%d:\n', numel(comp_fz_signed));
fprintf('     F_lc = %.4f*fz_signed + (%.5f)   R^2=%.5f\n', comp_a2, comp_b2, comp_r2_2);
fprintf('\n4. UR vs known weight (F_true), raw fz, n=%d:\n', numel(truth_fz));
fprintf('     pooled : F_true = %.4f*fz + (%.5f)   R^2=%.5f\n', truth_a, truth_b, truth_r2);
fprintf('     posz   : F_true = %.4f*fz + (%.5f)   R^2=%.5f\n', truth_posz_a, truth_posz_b, truth_posz_r2);
fprintf('     negz   : F_true = %.4f*fz + (%.5f)   R^2=%.5f\n', truth_negz_a, truth_negz_b, truth_negz_r2);
fprintf('\n5. UR vs known weight (F_true), sign-corrected fz, n=%d:\n', numel(truth_fz_signed));
fprintf('     F_true = %.4f*fz_signed + (%.5f)   R^2=%.5f\n', truth_a2, truth_b2, truth_r2_2);
fprintf('%s\n', repmat('=', 1, 78));


%% ============================= Local functions =============================

function entries = discover_entries(log_dir, instrument, standard_weights_g)
% DISCOVER_ENTRIES  Find fzcal_<instrument>_<direction>_<weight>g_<ts>.csv
% files and parse direction/weight/timestamp out of each filename.

    files = dir(fullfile(log_dir, sprintf('fzcal_%s_*.csv', instrument)));
    expr = ['fzcal_' instrument '_(?<direction>posz|negz)_' ...
            '(?<weight>\d+(\.\d+)?)g_(?<ts>\d{8}_\d{6})\.csv$'];

    entries_cell = {};
    for k = 1:numel(files)
        tok = regexp(files(k).name, expr, 'names');
        if isempty(tok)
            continue
        end
        weight_g = str2double(tok.weight);
        [~, nearest_idx] = min(abs(standard_weights_g - weight_g));

        e.instrument = instrument;
        e.direction = tok.direction;
        e.weight_g = weight_g;
        e.nominal_weight_g = standard_weights_g(nearest_idx);
        e.ts = tok.ts;
        e.csv_path = fullfile(files(k).folder, files(k).name);
        e.meta_path = strrep(e.csv_path, '.csv', '_meta.json');

        entries_cell{end + 1} = e; %#ok<AGROW>
    end

    if isempty(entries_cell)
        entries = struct([]);
    else
        entries = [entries_cell{:}];
    end
end


function kept = dedupe_latest(entries)
% DEDUPE_LATEST  Group entries by (instrument, direction, nominal weight)
% and keep only the chronologically-last file in each group.

    keys = cell(1, numel(entries));
    for i = 1:numel(entries)
        keys{i} = sprintf('%s|%s|%g', entries(i).instrument, entries(i).direction, ...
                           entries(i).nominal_weight_g);
    end
    unique_keys = unique(keys);

    kept_cell = cell(1, numel(unique_keys));
    for k = 1:numel(unique_keys)
        idx = find(strcmp(keys, unique_keys{k}));
        group = entries(idx);
        [~, order] = sort({group.ts});
        group = group(order);

        if numel(group) > 1
            dropped_labels = cell(1, numel(group) - 1);
            for j = 1:numel(group) - 1
                dropped_labels{j} = sprintf('%gg@%s', group(j).weight_g, group(j).ts);
            end
            fprintf('[dedupe] %-12s %-4s %5.0fg: keeping %gg@%s (dropping %s)\n', ...
                    group(end).instrument, group(end).direction, group(end).nominal_weight_g, ...
                    group(end).weight_g, group(end).ts, strjoin(dropped_labels, ', '));
        end
        kept_cell{k} = group(end);
    end

    if isempty(kept_cell)
        kept = struct([]);
    else
        kept = [kept_cell{:}];
    end
end


function entry = find_entry(entries, direction, weight_g)
% FIND_ENTRY  Return the single deduped entry matching (direction,
% nominal weight), or [] if none exists.

    for i = 1:numel(entries)
        if strcmp(entries(i).direction, direction) && entries(i).nominal_weight_g == weight_g
            entry = entries(i);
            return
        end
    end
    entry = [];
end


function s = find_session(sessions, direction, weight_g)
% FIND_SESSION  Return the single loaded session struct matching
% (direction, nominal weight), or [] if none exists.

    for i = 1:numel(sessions)
        if strcmp(sessions(i).direction, direction) && sessions(i).nominal_weight_g == weight_g
            s = sessions(i);
            return
        end
    end
    s = [];
end


function s = load_session(entry, G)
% LOAD_SESSION  Read one calibration CSV + its meta json, and compute the
% baseline and loaded means for fz and ai0. The baseline is a known,
% non-zero load (load cell + holder/hook hardware mass), not a zero
% reference.

    meta = jsondecode(fileread(entry.meta_path));
    T = readtable(entry.csv_path);

    is_loaded = T.loaded == 1;
    is_base = T.loaded == 0;

    s = entry;
    s.tilt_deg = meta.tilt_from_vertical_deg;
    s.fz_base_mean = mean(T.fz(is_base));
    s.fz_load_mean = mean(T.fz(is_loaded));
    s.ai0_base_mean = mean(T.ai0(is_base));
    s.ai0_load_mean = mean(T.ai0(is_loaded));

    tilt_rad = deg2rad(meta.tilt_from_vertical_deg);
    hardware_g = extra_hardware_futek_direct(entry.direction);
    total_g = entry.weight_g + hardware_g;

    s.F_true_base = (hardware_g / 1000) * G * cos(tilt_rad);
    s.F_true = (total_g / 1000) * G * cos(tilt_rad);
    s.F_signed = ai0_sign(entry.direction) * s.F_true;
    s.F_signed_base = ai0_sign(entry.direction) * s.F_true_base;

    % Separate ground truth for the UR sensor itself in this SAME rig: the
    % UR holds up the load cell's own body too, not just what the load
    % cell feels (see extra_hardware_futek_direct_ur).
    hardware_g_ur = extra_hardware_futek_direct_ur(entry.direction);
    total_g_ur = entry.weight_g + hardware_g_ur;
    s.F_true_ur_base = (hardware_g_ur / 1000) * G * cos(tilt_rad);
    s.F_true_ur = (total_g_ur / 1000) * G * cos(tilt_rad);
    s.F_signed_ur_base = ai0_sign(entry.direction) * s.F_true_ur_base;
    s.F_signed_ur = ai0_sign(entry.direction) * s.F_true_ur;
end


function sorted = sort_sessions(sessions)
% SORT_SESSIONS  Order a session struct array by direction then weight_g.

    keys = cell(1, numel(sessions));
    for i = 1:numel(sessions)
        keys{i} = sprintf('%s_%08.2f', sessions(i).direction, sessions(i).weight_g);
    end
    [~, order] = sort(keys);
    sorted = sessions(order);
end


function [t, is_loaded, fz, ai0] = load_raw_series(csv_path)
% LOAD_RAW_SERIES  Read the full raw time series for one session CSV --
% both fz (UR robot) and ai0 (load cell).

    T = readtable(csv_path);
    t = T.timestamp - T.timestamp(1);
    is_loaded = T.loaded == 1;
    fz = T.fz;
    ai0 = T.ai0;
end


function [m, c, r2, rmse] = linfit(x, y)
% LINFIT  Ordinary least-squares line  y = m*x + c , plus R^2 and RMSE.

    x = x(:);
    y = y(:);
    p = polyfit(x, y, 1);
    m = p(1);
    c = p(2);

    y_pred = m * x + c;
    ss_res = sum((y - y_pred) .^ 2);
    ss_tot = sum((y - mean(y)) .^ 2);
    r2 = 1 - ss_res / ss_tot;
    rmse = sqrt(mean((y - y_pred) .^ 2));
end


function sgn = ai0_sign(direction)
% AI0_SIGN  Load-cell voltage sign convention: +z (posz) pushes the
% bridge voltage down, -z (negz) pulls it up.

    if strcmp(direction, 'posz')
        sgn = -1;
    else
        sgn = 1;
    end
end


function extra_g = extra_hardware_futek_direct(direction)
% EXTRA_HARDWARE_FUTEK_DIRECT  Hardware mass (g) felt by the load cell's
% OWN reading in the futek_direct rig, on top of the nominal test weight:
% only what's mounted ABOVE the load cell in the load path -- the holder
% (7 g, posz) or the hook (4 g, negz -- a DIFFERENT, heavier hook than the
% 1 g one used in ur_only).

    if strcmp(direction, 'posz')
        extra_g = 7;
    else
        extra_g = 4;
    end
end


function extra_g = extra_hardware_futek_direct_ur(direction)
% EXTRA_HARDWARE_FUTEK_DIRECT_UR  Hardware mass (g) felt by the UR sensor
% itself in the SAME futek_direct rig: the UR holds up everything below
% it -- the 3D-printed coupler (15 g) + 4 attachment screws (21 g) + the
% load cell's own body (7 g) = 43 g, common to both directions, plus the
% holder (7 g, posz) or the hook (4 g, negz) above the load cell. Used
% for ground truth the UR's fz is compared against directly (bypassing
% the load cell) -- NOT for the ai0 fit.

    if strcmp(direction, 'posz')
        extra_g = 50;
    else
        extra_g = 47;
    end
end


function c = weight_color(weight_g)
% WEIGHT_COLOR  Fixed color per nominal calibration weight, so the same
% weight always plots in the same color across figures.

    switch round(weight_g)
        case 5,   c = [0.12 0.47 0.71];
        case 10,  c = [1.00 0.50 0.05];
        case 20,  c = [0.17 0.63 0.17];
        case 50,  c = [0.84 0.15 0.16];
        case 100, c = [0.58 0.40 0.74];
        case 200, c = [0.55 0.34 0.29];
        otherwise, c = [0.2 0.2 0.2];
    end
end
