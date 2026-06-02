% force_comparison.m
% Compares UR5 robot force (fz) and FUTEK load cell (AI0 → N)
% for three experimental cases: dome_empty, dome_solid, solid.
%
% Signal conditioning (all channels):
%   zero_ref = mean(raw[rest rows in session window])  →  rest = 0, pressing > 0
%   Crop: display from first pressing event onward.
%
% FUTEK calibration (from analyze_session.py):
%   F_N = −(V_ai0 − 5.0) × (44.482 / 5.0)   [8.896 N/V]
%
% Figures:
%   Fig 1 – Raw signals (full session)
%   Fig 2 – Conditioned fz + FUTEK  (shared y-scale)
%   Fig 3 – Per-point mean ± std    (shared y-scale)
%   Fig 4 – UR5 vs FUTEK scatter
%   Fig 5 – All 6 UR5 channels (fx fy fz tx ty tz, shared scales per unit)
%
% Usage:
%   cd  <repo_root>/Matlab/force_analysis
%   run force_comparison.m
% -----------------------------------------------------------------------

clear; clc; close all;

% ── File paths ──────────────────────────────────────────────────────────
REPO_ROOT = fullfile(fileparts(mfilename('fullpath')), '..', '..');
LOG_DIR   = fullfile(REPO_ROOT, 'Integration_2', 'logs');

FILES = { ...
    fullfile(LOG_DIR, 'dome_empty_may_28_9mm_session_20260528_225530.csv'), ...
    fullfile(LOG_DIR, 'dome_solid_may_28_9mm_session_20260528_232207.csv'), ...
    fullfile(LOG_DIR, 'solid_may_28_9mm_session_20260528_233223.csv')       ...
};

CASE_LABELS = {'Dome — Empty', 'Dome — Solid', 'Solid (no dome)'};
CASE_COLORS = { ...
    [0.20, 0.60, 0.86], ...
    [0.91, 0.30, 0.24], ...
    [0.18, 0.80, 0.44]  ...
};

% ── FUTEK calibration ───────────────────────────────────────────────────
AI0_ZERO_V     = 5.0;
LOADCELL_MAX_N = 10.0 * 4.44822;
N_PER_V        = LOADCELL_MAX_N / 5.0;
ai0_to_N       = @(v) -(v - AI0_ZERO_V) * N_PER_V;

N_POINTS  = 19;
T_SKIP    = 5.0;                                       % seconds to skip at start (startup ramp)
FT_NAMES  = {'fx','fy','fz','tx','ty','tz'};          % all 6 channels
FT_LABELS = {'F_x (N)','F_y (N)','F_z (N)', ...
             'T_x (N·m)','T_y (N·m)','T_z (N·m)'};

% ── Load and condition all sessions ─────────────────────────────────────
S = struct();

for k = 1:3
    T        = readtable(FILES{k}, 'VariableNamingRule', 'preserve');
    pressing = T.ur5_pressing == 1;

    first_press_idx = find(pressing, 1, 'first');
    if isempty(first_press_idx); first_press_idx = 1; end

    idx    = first_press_idx : height(T);
    t_full = T.timestamp - T.timestamp(1);
    t_crop = t_full(idx) - t_full(first_press_idx);

    % Raw (for Fig 1)
    S(k).fz_raw    = T.fz;
    S(k).futek_raw = ai0_to_N(T.ai0);
    S(k).t_full    = t_full;

    % Zero reference: mean of REST rows within session window.
    % Using mean(rest) rather than min(session) avoids pressing transients
    % that dip slightly below the resting baseline and cause a false offset.
    futek_raw   = ai0_to_N(T.ai0);
    rest_in_win = T.ur5_pressing(idx) == 0;   % logical mask within session window

    fz_zero    = mean(T.fz(idx(rest_in_win)));
    futek_zero = mean(futek_raw(idx(rest_in_win)));

    % Full-session conditioned (same zero ref → correct baseline, full time axis)
    S(k).fz_cond_full    = T.fz      - fz_zero;
    S(k).futek_cond_full = futek_raw - futek_zero;

    % Cropped versions (from first press) — used only for per-point stats
    S(k).fz_cond    = T.fz(idx)      - fz_zero;
    S(k).futek_cond = futek_raw(idx) - futek_zero;

    % All 6 force/torque channels — zero ref = mean of rest rows
    for c = 1:6
        raw_c   = T.(FT_NAMES{c});
        ch_zero = mean(raw_c(idx(rest_in_win)));
        S(k).(FT_NAMES{c})           = raw_c      - ch_zero;  % full session
        S(k).([FT_NAMES{c} '_crop']) = raw_c(idx) - ch_zero;  % cropped (stats)
    end

    S(k).t_crop          = t_crop;
    S(k).pressing_full   = pressing;          % full session mask
    S(k).pressing        = pressing(idx);     % cropped mask (stats)
    S(k).ur5_point       = T.ur5_point(idx);

    fprintf('[%s]  fz_zero=%.3f N (rest mean, n=%d)  futek_zero=%.3f N  crop@%d\n', ...
            CASE_LABELS{k}, fz_zero, sum(rest_in_win), futek_zero, first_press_idx);
end

% ── Global y-maxima (computed once, applied to all matching subplots) ───
% Only use t >= T_SKIP to exclude the startup ramp spike.
global_ymax_fz    = 0;
global_ymax_force = 0;
global_ymax_torque = 0;

for k = 1:3
    t5 = S(k).t_full >= T_SKIP;   % mask: skip first T_SKIP seconds

    % fz / FUTEK
    global_ymax_fz = max(global_ymax_fz, ...
                         max([S(k).fz_cond_full(t5); S(k).futek_cond_full(t5)]));

    % Force channels (fx, fy, fz) — N
    for c = 1:3
        global_ymax_force = max(global_ymax_force, max(S(k).(FT_NAMES{c})(t5)));
    end

    % Torque channels (tx, ty, tz) — N·m
    for c = 4:6
        global_ymax_torque = max(global_ymax_torque, max(S(k).(FT_NAMES{c})(t5)));
    end
end

global_ymax_fz     = global_ymax_fz     * 1.12;
global_ymax_force  = global_ymax_force  * 1.12;
global_ymax_torque = global_ymax_torque * 1.12;

fprintf('\nGlobal y-max — fz/FUTEK: %.2f N  |  forces: %.2f N  |  torques: %.4f N·m\n\n', ...
        global_ymax_fz, global_ymax_force, global_ymax_torque);


% ═══════════════════════════════════════════════════════════════════════
% Fig 1 – Raw signals (full session, no processing)
% ═══════════════════════════════════════════════════════════════════════
fig1 = figure('Name','Raw Signals','NumberTitle','off', ...
              'Units','normalized','Position',[0.04 0.04 0.92 0.88]);
tiledlayout(3, 2, 'TileSpacing','compact','Padding','compact');

for k = 1:3
    clr = CASE_COLORS{k};
    nexttile;
    hold on; box on;
    plot(S(k).t_full, S(k).fz_raw, 'Color', clr, 'LineWidth', 1.0);
    yline(0,'k--','LineWidth',0.7);
    xlabel('Time (s)','FontSize',9); ylabel('f_z  (N)  raw','FontSize',9);
    title(sprintf('%s — UR5 F_z  (raw)', CASE_LABELS{k}),'FontSize',10,'FontWeight','bold');
    grid on;

    nexttile;
    hold on; box on;
    plot(S(k).t_full, S(k).futek_raw, 'Color', clr, 'LineWidth', 1.0);
    yline(0,'k--','LineWidth',0.7);
    xlabel('Time (s)','FontSize',9); ylabel('F  (N)  raw','FontSize',9);
    title(sprintf('%s — FUTEK  (raw, V→N)', CASE_LABELS{k}),'FontSize',10,'FontWeight','bold');
    grid on;
end
sgtitle({'Raw Force Signals — Full Session (no conditioning)', ...
         '\rmUR5 col: raw f_z from RTDE  |  FUTEK col: AI0 voltage converted to N'}, ...
        'FontSize',13,'FontWeight','bold');


% ═══════════════════════════════════════════════════════════════════════
% Fig 2 – Conditioned fz + FUTEK, shared y-scale
% ═══════════════════════════════════════════════════════════════════════
fig2 = figure('Name','Conditioned Signals — Pressing Window','NumberTitle','off', ...
              'Units','normalized','Position',[0.04 0.04 0.92 0.88]);
tiledlayout(3, 2, 'TileSpacing','compact','Padding','compact');

for k = 1:3
    t = S(k).t_full;  pressing = S(k).pressing_full;  clr = CASE_COLORS{k};

    t_end = t(end);

    nexttile; hold on; box on;
    fill_press_bg(t, pressing, [0.95 0.95 0.80]);
    plot(t, S(k).fz_cond_full, 'Color', clr, 'LineWidth', 1.2);
    yline(0,'k--','LineWidth',0.8);
    xlabel('Time (s)','FontSize',9); ylabel('F_z  (N)','FontSize',9);
    title(sprintf('%s — UR5 F_z', CASE_LABELS{k}),'FontSize',10,'FontWeight','bold');
    xlim([T_SKIP t_end]); ylim([0 global_ymax_fz]); grid on;
    legend({'pressing','UR5 F_z'},'Location','best','FontSize',8);

    nexttile; hold on; box on;
    fill_press_bg(t, pressing, [0.95 0.95 0.80]);
    plot(t, S(k).futek_cond_full, 'Color', clr, 'LineWidth', 1.2);
    yline(0,'k--','LineWidth',0.8);
    xlabel('Time (s)','FontSize',9); ylabel('F  (N)','FontSize',9);
    title(sprintf('%s — FUTEK', CASE_LABELS{k}),'FontSize',10,'FontWeight','bold');
    xlim([T_SKIP t_end]); ylim([0 global_ymax_fz]); grid on;
    legend({'pressing','FUTEK'},'Location','best','FontSize',8);
end
sgtitle({'Conditioned Signals  (startup ramp excluded, shared y-scale)', ...
         sprintf('\\rmrest=0 · pressing>0 · y_{max}=%.2f N  (t>%.0f s)', global_ymax_fz, T_SKIP)}, ...
        'FontSize',13,'FontWeight','bold');


% ═══════════════════════════════════════════════════════════════════════
% Fig 3 – Per-point mean ± std, shared y-scale
% ═══════════════════════════════════════════════════════════════════════
fig3 = figure('Name','Per-Point Force','NumberTitle','off', ...
              'Units','normalized','Position',[0.04 0.04 0.92 0.88]);
tiledlayout(3, 2, 'TileSpacing','compact','Padding','compact');

xp    = 1:N_POINTS;
xlbls = arrayfun(@(n) sprintf('P%d',n), xp, 'UniformOutput', false);

for k = 1:3
    pressing = S(k).pressing;  pt = S(k).ur5_point;  clr = CASE_COLORS{k};

    fz_mean = nan(N_POINTS,1);  fz_std = nan(N_POINTS,1);
    ft_mean = nan(N_POINTS,1);  ft_std = nan(N_POINTS,1);
    for p = 1:N_POINTS
        mask = pressing & (pt == p);
        if any(mask)
            fz_mean(p) = mean(S(k).fz_cond(mask),    'omitnan');
            fz_std(p)  = std(S(k).fz_cond(mask),     'omitnan');
            ft_mean(p) = mean(S(k).futek_cond(mask),  'omitnan');
            ft_std(p)  = std(S(k).futek_cond(mask),   'omitnan');
        end
    end

    nexttile; hold on; box on;
    bar(xp, fz_mean, 0.6, 'FaceColor', clr, 'EdgeColor','none','FaceAlpha',0.85);
    errorbar(xp, fz_mean, fz_std, 'k.','LineWidth',1.0,'CapSize',4);
    yline(0,'k--','LineWidth',0.8);
    xlabel('Contact Point','FontSize',9); ylabel('F_z  (N)','FontSize',9);
    title(sprintf('%s — UR5 per Point', CASE_LABELS{k}),'FontSize',10,'FontWeight','bold');
    xticks(xp); xticklabels(xlbls); xtickangle(45);
    ylim([0 global_ymax_fz]); grid on;

    nexttile; hold on; box on;
    bar(xp, ft_mean, 0.6, 'FaceColor', clr, 'EdgeColor','none','FaceAlpha',0.85);
    errorbar(xp, ft_mean, ft_std, 'k.','LineWidth',1.0,'CapSize',4);
    yline(0,'k--','LineWidth',0.8);
    xlabel('Contact Point','FontSize',9); ylabel('F  (N)','FontSize',9);
    title(sprintf('%s — FUTEK per Point', CASE_LABELS{k}),'FontSize',10,'FontWeight','bold');
    xticks(xp); xticklabels(xlbls); xtickangle(45);
    ylim([0 global_ymax_fz]); grid on;
end
sgtitle(sprintf('Per-Contact-Point Mean ± Std  (shared y_{max} = %.2f N)', global_ymax_fz), ...
        'FontSize',13,'FontWeight','bold');


% ═══════════════════════════════════════════════════════════════════════
% Fig 4 – UR5 vs FUTEK scatter
% ═══════════════════════════════════════════════════════════════════════
fig4 = figure('Name','UR5 vs FUTEK Scatter','NumberTitle','off', ...
              'Units','normalized','Position',[0.15 0.10 0.50 0.60]);
ax4 = axes(fig4);
hold(ax4,'on'); box(ax4,'on');

all_v = [];
for k = 1:3
    mask = S(k).pressing_full;
    fz_p = S(k).fz_cond_full(mask);
    ft_p = S(k).futek_cond_full(mask);
    scatter(ax4, fz_p, ft_p, 8, CASE_COLORS{k}, 'filled', ...
            'MarkerFaceAlpha',0.4,'DisplayName',CASE_LABELS{k});
    all_v = [all_v; fz_p(:); ft_p(:)];  %#ok<AGROW>
end
lim_hi = global_ymax_fz;
plot(ax4, [0 lim_hi],[0 lim_hi],'k--','LineWidth',1.2,'DisplayName','1:1 line');
xlabel(ax4,'UR5  F_z  (N)','FontSize',11);
ylabel(ax4,'FUTEK  F  (N)','FontSize',11);
title(ax4,'UR5 vs FUTEK — Pressing Samples (conditioned)','FontSize',12,'FontWeight','bold');
legend(ax4,'Location','northwest','FontSize',10);
xlim(ax4,[0 lim_hi]); ylim(ax4,[0 lim_hi]);
grid(ax4,'on'); axis(ax4,'equal');


% ═══════════════════════════════════════════════════════════════════════
% Fig 5 – All 6 UR5 channels (fx fy fz tx ty tz), 3 cases × 6 cols
%          Forces share one y-scale, torques share another.
% ═══════════════════════════════════════════════════════════════════════
fig5 = figure('Name','UR5 — All 6 Force/Torque Channels','NumberTitle','off', ...
              'Units','normalized','Position',[0.02 0.02 0.98 0.92]);
tl5 = tiledlayout(3, 6, 'TileSpacing','compact','Padding','compact');

for k = 1:3
    t        = S(k).t_full;
    pressing = S(k).pressing_full;
    clr      = CASE_COLORS{k};

    t_end = t(end);
    for c = 1:6
        nexttile;
        hold on; box on;
        fill_press_bg(t, pressing, [0.95 0.95 0.80]);
        plot(t, S(k).(FT_NAMES{c}), 'Color', clr, 'LineWidth', 1.0);
        yline(0,'k--','LineWidth',0.6);

        xlim([T_SKIP t_end]);
        if c <= 3
            ylim([0 global_ymax_force]);
        else
            ylim([0 global_ymax_torque]);
        end

        grid on;
        xlabel('Time (s)','FontSize',7);
        ylabel(FT_LABELS{c},'FontSize',7);

        if k == 1
            title(FT_LABELS{c},'FontSize',9,'FontWeight','bold');
        end
        if c == 1
            text(0.02, 0.90, CASE_LABELS{k}, 'Units','normalized', ...
                 'FontSize',8,'FontWeight','bold','Color',clr);
        end
    end
end

title(tl5, ...
    {sprintf('UR5 All 6 Channels — Conditioned  (t > %.0f s, startup ramp excluded)', T_SKIP), ...
     sprintf('\\rmForces y_{max}=%.2f N  |  Torques y_{max}=%.4f N·m  |  yellow = pressing', ...
             global_ymax_force, global_ymax_torque)}, ...
    'FontSize', 12, 'FontWeight', 'bold');


% ── Save ────────────────────────────────────────────────────────────────
OUT_DIR = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~exist(OUT_DIR,'dir'); mkdir(OUT_DIR); end

exportgraphics(fig1, fullfile(OUT_DIR,'raw_signals.pdf'),        'ContentType','vector');
exportgraphics(fig2, fullfile(OUT_DIR,'conditioned_signals.pdf'),'ContentType','vector');
exportgraphics(fig3, fullfile(OUT_DIR,'per_point.pdf'),          'ContentType','vector');
exportgraphics(ax4,  fullfile(OUT_DIR,'scatter.pdf'),            'ContentType','vector');
exportgraphics(fig5, fullfile(OUT_DIR,'ur5_all_channels.pdf'),   'ContentType','vector');

fprintf('Saved to  %s\n', OUT_DIR);


% ── Helpers ─────────────────────────────────────────────────────────────

function fill_press_bg(t, mask, clr)
    if isempty(t) || ~any(mask); return; end
    starts = find(diff([0; double(mask(:))]) ==  1);
    ends   = find(diff([double(mask(:)); 0]) == -1);
    yl = ylim();
    for i = 1:numel(starts)
        patch([t(starts(i)) t(ends(i)) t(ends(i)) t(starts(i))], ...
              [yl(1) yl(1) yl(2) yl(2)], clr, ...
              'EdgeColor','none','FaceAlpha',0.45,'HandleVisibility','off');
    end
end
