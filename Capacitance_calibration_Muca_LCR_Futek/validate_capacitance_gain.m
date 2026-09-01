% =========================================================================
% validate_capacitance_gain.m
%
% Justification figure for the muca-count -> capacitance GAIN used by
% mucaboard_data_raw/matlab/capacitance_19points_analysis.m, and an
% honest test of whether applying it to touch data is defensible.
%
% Four panels:
%   A  The calibration itself. Known capacitors wired to muca inputs,
%      true value measured on the LCR-6100 at the same instant. Fit is
%      excellent (R^2 ~ 0.99) and repeatable across 5 different inputs
%      per capacitor -- the gain is well determined WHERE IT WAS MEASURED.
%
%   B  The extrapolation gap. Panel A was measured at raw readings of
%      about -337 to -1305 counts. Real touch deltas in
%      mucaboard_data_raw span roughly +2 to +24 counts. The two ranges
%      do not overlap at all -- applying the gain to touch data is an
%      extrapolation of 1-2 orders of magnitude, and of opposite sign.
%
%   C  Head-to-head validation against ground truth. For every point and
%      surface, the gain's PREDICTED dCp is plotted against the dCp the
%      LCR actually MEASURED on the sensor over the same untouched->
%      pressed transition (the LCR sweep's 'locate' phase gives a true
%      untouched reference; 'post' returns to it within ~0.0001 pF, so it
%      is a clean zero). Perfect agreement would lie on the 1:1 line.
%
%   D  Fitting the gain directly in the operating regime instead --
%      measured dCp vs measured raw delta, across all 19 points x 3
%      surfaces. If muca counts were proportional to capacitance change
%      here, this would be a tight line through the origin.
%
% Run: octave-cli validate_capacitance_gain.m   (or open in MATLAB)
% =========================================================================

clear; close all; clc;

HERE          = fileparts(mfilename('fullpath'));
REPO_ROOT     = fullfile(HERE, '..');
MUCA_TOOLKIT  = fullfile(REPO_ROOT, 'mucaboard_data', 'matlab');
MUCA_RAW_LOGS = fullfile(REPO_ROOT, 'mucaboard_data_raw', 'logs');
CAPMEAS_LOGS  = fullfile(REPO_ROOT, 'Capacitance_measurement', 'logs');
COMBO_LOGS    = fullfile(REPO_ROOT, 'capacitance_combination_calibration', 'logs');
RESULTS_DIR   = fullfile(HERE, 'results');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end
addpath(MUCA_TOOLKIT);

TRUE_TO_CODE  = [8,4,1,13,9,5,2,17,14,10,6,3,18,15,11,7,19,16,12];
N_ITERATIONS  = 10;
SURFACE_NAMES = {'flat', 'solid', 'hollow'};
SURFACE_COLORS = [0.9490 0.7804 0.3608; 0.8784 0.4941 0.2353; 0.4784 0.3765 0.2510];

MUCA_RAW_FILES = { ...
    fullfile(MUCA_RAW_LOGS, 'flat_sensor_19_points_10_iterations_session_20260826_155255.csv'), ...
    fullfile(MUCA_RAW_LOGS, 'solid_sensor_19_points_10_iterations_session_20260826_165651.csv'), ...
    fullfile(MUCA_RAW_LOGS, 'hollow_sensor_iterations_all_session_20260826_190632.csv') ...
};
LCR_FILES = { ...
    fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260828_193051_flat_sensor.csv'), ...
    fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260828_162718_solid_sensor.csv'), ...
    fullfile(CAPMEAS_LOGS, 'two_point_iterations_ALL19_20260825_202746_hollow_sensor.csv') ...
};
COMBO_FILE = fullfile(COMBO_LOGS, 'flat_calibration_muca_lcr_session_20260826_225636.csv');

function v = to_signed16(v)
    v(v > 32767) = v(v > 32767) - 65536;
end

function out = nanmean_cols(M)
    out = nan(1, size(M, 2));
    for c = 1:size(M, 2)
        col = M(:, c); col = col(~isnan(col));
        if ~isempty(col), out(c) = mean(col); end
    end
end

function [m, b, r2] = linfit(x, y)
    x = x(:); y = y(:);
    p = polyfit(x, y, 1); m = p(1); b = p(2);
    yf = polyval(p, x);
    r2 = 1 - sum((y - yf).^2) / max(sum((y - mean(y)).^2), eps);
end

fprintf('%s\n', repmat('=', 1, 74));
fprintf('  VALIDATION of the muca-count -> capacitance GAIN\n');
fprintf('%s\n', repmat('=', 1, 74));

%% ---- PANEL A data: the combination calibration -------------------------
fid = fopen(COMBO_FILE, 'r');
hl = fgetl(fid); hdr = strsplit(hl, ',', 'CollapseDelimiters', false);
Cc = textscan(fid, repmat('%s',1,numel(hdr)), 'Delimiter', ',', 'Whitespace', '');
fclose(fid);
cg = @(n) Cc{find(strcmp(hdr,n),1)};
c_phase = cg('phase'); c_label = cg('combination_label'); c_point = cg('point_label');
c_cp = str2double(cg('Cp_pF'));
c_cells = nan(numel(c_phase),19);
for k=1:19, c_cells(:,k) = str2double(cg(sprintf('cell_%d',k))); end

cal_raw = []; cal_cp = []; cal_combo = {};
labels = unique(c_label);
for i = 1:numel(labels)
    lb = labels{i};
    m_lcr = strcmp(c_phase,'lcr') & strcmp(c_label,lb);
    if ~any(m_lcr), continue; end
    cp_true = mean(c_cp(m_lcr),'omitnan');
    pts = unique(c_point(strcmp(c_phase,'muca') & strcmp(c_label,lb)));
    for j = 1:numel(pts)
        m = strcmp(c_phase,'muca') & strcmp(c_label,lb) & strcmp(c_point,pts{j});
        if ~any(m), continue; end
        cm = nanmean_cols(to_signed16(c_cells(m,:)));
        [ra, ~] = min(cm);
        if ra > -20, continue; end
        cal_raw(end+1) = ra; cal_cp(end+1) = cp_true; cal_combo{end+1} = lb; %#ok<AGROW>
    end
end
[GAIN, cal_b, cal_r2] = linfit(cal_raw, cal_cp);
fprintf('\n[A] Combination calibration: Cp = %.6f*raw + %.4f   R2=%.4f  n=%d\n', ...
    GAIN, cal_b, cal_r2, numel(cal_raw));
fprintf('    calibration raw range: %.0f .. %.0f counts\n', min(cal_raw), max(cal_raw));

%% ---- PANEL C/D data: muca raw delta vs LCR-measured dCp ---------------
draw_all = []; dcp_all = []; surf_idx = [];
for s = 1:3
    rows_m = {};
    fid = fopen(MUCA_RAW_FILES{s},'r');
    hl = fgetl(fid); cn = strsplit(hl,',','CollapseDelimiters',false);
    while true
        L = fgetl(fid); if ~ischar(L), break; end
        rows_m{end+1} = strsplit(L,',','CollapseDelimiters',false); %#ok<AGROW>
    end
    fclose(fid);
    dt = vertcat(rows_m{:});
    ci = @(n) find(strcmp(cn,n)); gn = @(n) str2double(dt(:,ci(n)));
    up = gn('ur5_point'); ri = gn('round_idx'); ph = dt(:,ci('phase'));
    rc = nan(size(dt,1),19);
    for k=1:19, rc(:,k) = to_signed16(gn(sprintf('cell_%d',k))); end
    cb = nan(1,19);
    for k=1:19, o = to_signed16(gn(sprintf('calib_%d',k))); cb(k)=o(1); end

    % LCR side
    fid = fopen(LCR_FILES{s},'r');
    hl = fgetl(fid); hdr2 = strsplit(hl,',','CollapseDelimiters',false);
    C2 = textscan(fid, repmat('%s',1,numel(hdr2)), 'Delimiter',',','Whitespace','');
    fclose(fid);
    g2 = @(n) C2{find(strcmp(hdr2,n),1)};
    l_pt = str2double(g2('point')); l_ph = g2('phase'); l_cp = str2double(g2('Cp_pF'));

    for true_pt = 1:19
        code = TRUE_TO_CODE(true_pt);
        mm = (up==code) & (ri>=0) & (ri<=N_ITERATIONS-1) & strcmp(ph,'hold');
        if ~any(mm), continue; end
        draw = mean(rc(mm,code),'omitnan') - cb(code);
        m_loc = (l_pt==code) & strcmp(l_ph,'locate');
        m_hld = (l_pt==code) & strcmp(l_ph,'hold');
        if ~any(m_loc) || ~any(m_hld), continue; end
        dcp = mean(l_cp(m_hld),'omitnan') - mean(l_cp(m_loc),'omitnan');
        draw_all(end+1)=draw; dcp_all(end+1)=dcp; surf_idx(end+1)=s; %#ok<AGROW>
    end
end
pred_all = GAIN * draw_all;
[dm, db, dr2] = linfit(draw_all, dcp_all);
fprintf('\n[B] Operating-regime raw deltas: %.1f .. %.1f counts  (NO overlap with A)\n', ...
    min(draw_all), max(draw_all));
fprintf('\n[C] Gain-predicted vs LCR-measured dCp:\n');
fprintf('    predicted mean %.4f pF   measured mean %.4f pF   ratio %.2fx\n', ...
    mean(pred_all), mean(dcp_all), mean(dcp_all)/mean(pred_all));
fprintf('    per-point ratio spans %.2fx .. %.2fx\n', ...
    min(dcp_all./pred_all), max(dcp_all./pred_all));
fprintf('\n[D] Direct fit in the operating regime:\n');
fprintf('    dCp = %.6f*draw + %.5f   R2=%.4f  n=%d\n', dm, db, dr2, numel(draw_all));
for s = 1:3
    k = surf_idx==s;
    [~,~,r2s] = linfit(draw_all(k), dcp_all(k));
    fprintf('      %-7s R2 = %.4f\n', SURFACE_NAMES{s}, r2s);
end

%% ---- FIGURE -------------------------------------------------------------
FONT = 'Helvetica';
fig = figure('Position',[60 60 1500 1050]);

% -- A: calibration fit
axA = axes('Parent',fig,'Position',[0.07 0.57 0.38 0.36]); hold(axA,'on'); box(axA,'on'); grid(axA,'on');
ulab = unique(cal_combo);
for i=1:numel(ulab)
    k = strcmp(cal_combo, ulab{i});
    % (no MarkerFaceAlpha -- unsupported by Octave's gnuplot toolkit)
    scatter(axA, cal_raw(k), cal_cp(k), 55, 'filled');
end
xl = linspace(min(cal_raw)*1.05, max(cal_raw)*0.9, 50);
plot(axA, xl, polyval([GAIN cal_b], xl), 'k-', 'LineWidth', 2);
xlabel(axA,'muca reading (signed counts)','FontName',FONT);
ylabel(axA,'LCR-measured Cp (pF)','FontName',FONT);
title(axA, sprintf('A. Calibration: Cp = %.6f*raw + %.3f   R^2=%.4f (n=%d)', GAIN, cal_b, cal_r2, numel(cal_raw)), ...
    'FontName',FONT,'FontSize',10);
text(axA, 0.03, 0.10, 'known capacitors wired to muca inputs;', 'Units','normalized','FontSize',8,'FontName',FONT);
text(axA, 0.03, 0.04, '5 different inputs per capacitor -> tight clusters', 'Units','normalized','FontSize',8,'FontName',FONT);

% -- B: extrapolation gap
axB = axes('Parent',fig,'Position',[0.56 0.57 0.38 0.36]); hold(axB,'on'); box(axB,'on'); grid(axB,'on');
plot(axB, [min(cal_raw) max(cal_raw)], [1 1], '-', 'Color',[0.2 0.4 0.8], 'LineWidth', 9);
plot(axB, [min(draw_all) max(draw_all)], [2 2], '-', 'Color',[0.85 0.33 0.10], 'LineWidth', 9);
set(axB,'YTick',[1 2],'YTickLabel',{'calibrated here','used here'});
ylim(axB,[0.5 2.5]);
xlabel(axB,'muca reading / delta (signed counts)','FontName',FONT);
title(axB,'B. The extrapolation gap: the two ranges do not overlap','FontName',FONT,'FontSize',10);
text(axB, mean([min(cal_raw) max(cal_raw)]), 1.22, sprintf('%.0f .. %.0f counts', min(cal_raw), max(cal_raw)), ...
    'HorizontalAlignment','center','FontSize',9,'FontName',FONT,'Color',[0.2 0.4 0.8]);
text(axB, mean([min(draw_all) max(draw_all)]), 2.22, sprintf('%.1f .. %.1f counts', min(draw_all), max(draw_all)), ...
    'HorizontalAlignment','center','FontSize',9,'FontName',FONT,'Color',[0.85 0.33 0.10]);

% -- C: predicted vs measured
axC = axes('Parent',fig,'Position',[0.07 0.08 0.38 0.38]); hold(axC,'on'); box(axC,'on'); grid(axC,'on');
hC = [];
for s=1:3
    k = surf_idx==s;
    hC(end+1) = scatter(axC, pred_all(k), dcp_all(k), 55, SURFACE_COLORS(s,:), 'filled'); %#ok<AGROW>
end
lims = [min([pred_all dcp_all])*1.1, 0];
hC(end+1) = plot(axC, lims, lims, 'k--', 'LineWidth', 1.5);
xlabel(axC,'dCp PREDICTED by the gain (pF)','FontName',FONT);
ylabel(axC,'dCp MEASURED by LCR on the sensor (pF)','FontName',FONT);
title(axC,'C. Validation vs ground truth: points fall far off 1:1','FontName',FONT,'FontSize',10);
% explicit label list -- Octave's gnuplot toolkit ignores DisplayName on scatter()
legend(axC, hC, [SURFACE_NAMES, {'1:1 (perfect agreement)'}], 'Location','southeast','FontSize',8);
axis(axC,'square');

% -- D: direct fit in the operating regime
axD = axes('Parent',fig,'Position',[0.56 0.08 0.38 0.38]); hold(axD,'on'); box(axD,'on'); grid(axD,'on');
hD = [];
for s=1:3
    k = surf_idx==s;
    hD(end+1) = scatter(axD, draw_all(k), dcp_all(k), 55, SURFACE_COLORS(s,:), 'filled'); %#ok<AGROW>
end
xd = linspace(min(draw_all), max(draw_all), 50);
hD(end+1) = plot(axD, xd, polyval([dm db], xd), 'k-', 'LineWidth', 2);
hD(end+1) = plot(axD, xd, GAIN*xd, 'r--', 'LineWidth', 1.8);
xlabel(axD,'muca raw delta, untouched -> pressed (counts)','FontName',FONT);
ylabel(axD,'dCp MEASURED by LCR (pF)','FontName',FONT);
title(axD, sprintf('D. Fit directly in the operating regime: R^2=%.3f -- no proportionality', dr2), ...
    'FontName',FONT,'FontSize',10);
legend(axD, hD, [SURFACE_NAMES, {sprintf('direct fit R^2=%.3f', dr2)}, {'combo-calibration gain'}], ...
    'Location','northeast','FontSize',8);

fig_suptitle(fig, 'Justification & validation of the muca-count -> capacitance gain');
save_fig(fig, fullfile(RESULTS_DIR,'capacitance_gain_validation'));

fprintf('\n  Saved: %s.(png/svg)\n', fullfile(RESULTS_DIR,'capacitance_gain_validation'));
fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('  DONE.\n');
fprintf('%s\n', repmat('=', 1, 74));
