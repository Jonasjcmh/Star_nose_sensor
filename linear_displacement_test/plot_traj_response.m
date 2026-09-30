% plot_traj_response.m — Multi-point trajectory | cell response over a pass
% =========================================================================
% For logs from trajectory_displacement.py (a ring of pads slid through
% out and back, 1 s hold at every pad, tip engaged the whole time). Values
% come from traj_timeline.m: session 'locate' baseline, every slide/hold
% piecewise-aligned across the 5 passes, median across passes.
%
% Three outputs per session, in results/trajectory/response/:
%
%   <tag>_heatmap  -- dC/C0 of every cell (rows) against time over one
%                     pass. Ring cells come first, in path order, so a
%                     sensor that tracks the tip shows a staircase; the
%                     black line is where the tip actually is (the row of
%                     the pad it sits on / is sliding between).
%   <tag>_traces   -- the ring cells as traces over the same pass, one
%                     colour per pad, the pad name marked at each hold.
%   <tag>_tracking -- per pad: % of holds (5 passes) where the hottest of
%                     the 19 cells is the pad under the tip, out and back.
%                     Also written as <tag>_tracking.csv, one row per hold.
% =========================================================================

clear; close all; clc;

HERE = fileparts(mfilename('fullpath'));
addpath(HERE);
addpath(fullfile(HERE, '..', 'Interdome_touch', 'matlab_analysis'));

% ── CONFIG ───────────────────────────────────────────────────────────────
SESSIONS = { ...
    'hollow_dome_5_iterations_external_diameter_session_20260930_191043.csv', 'ring_external'; ...
    'hollow_dome_5_iterations_internal_diameter_session_20260930_193505.csv', 'ring_internal'};
T_PRE  = 6.0;
T_POST = 3.0;
BIN_S  = 0.5;
LAB = {'a1','a2','a3','b1','b2','b3','b4','c1','c2','c3', ...
       'c4','c5','d2','d3','d4','d5','e3','e4','e5'};
SLIDE_OUT_RGB  = [0.831 0.855 0.878];
SLIDE_BACK_RGB = [0.851 0.878 0.851];
HOLD_RGB       = [0.945 0.945 0.945];
AX_W = 1700;
KEEP_FIGURES_OPEN = false;

LOG_DIR     = fullfile(HERE, 'logs');
RESULTS_DIR = fullfile(HERE, 'results', 'trajectory', 'response');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end

for si = 1:size(SESSIONS, 1)
    csv_name = SESSIONS{si, 1};
    tag      = SESSIONS{si, 2};
    csv_path = fullfile(LOG_DIR, csv_name);
    fprintf('\n%s\n  SESSION: %s\n%s\n', repmat('=', 1, 70), tag, repmat('=', 1, 70));
    if ~exist(csv_path, 'file')
        warning('plot_traj_response:missing', 'skipping %s', csv_name);
        continue;
    end
    T = traj_timeline(csv_path, T_PRE, T_POST, BIN_S);
    ring = unique(T.path_out, 'stable');           % P1..PN, no repeat
    nring = numel(ring);
    k_ring = cellfun(@(p) find(strcmp(LAB, p), 1), ring);
    k_rest = setdiff(1:19, k_ring);
    order  = [k_ring, k_rest];
    x = T.t_centre + T_PRE;                         % plotted axis starts at 0
    x_stop = T.t_stop + T_PRE;
    fprintf('  %d passes | ring %s | pass %.0f s\n', T.n_pass, ...
        strjoin(T.path_out, '>'), T.t_core_end);

    % tip row over time: the row of the pad under / between the tip
    % A closed ring (last pad -> P1) WRAPS: the line leaves the bottom of the
    % ring block and re-enters at the top, instead of cutting across it.
    tip_t = T_PRE; tip_r = 1;
    for j = 1:numel(T.blocks)
        B = T.blocks(j);
        rf = find(strcmp(ring, B.from), 1); rt = find(strcmp(ring, B.to), 1);
        t2 = T_PRE + [B.t_start, B.t_end];
        if abs(rt - rf) == nring - 1 && nring > 2
            tm = mean(t2);
            if rf == nring                       % bottom -> wrap -> top
                tip_t = [tip_t, t2(1), tm, NaN, tm, t2(2)];            %#ok<AGROW>
                tip_r = [tip_r, rf, nring + 0.5, NaN, 0.5, rt];        %#ok<AGROW>
            else                                 % top -> wrap -> bottom
                tip_t = [tip_t, t2(1), tm, NaN, tm, t2(2)];            %#ok<AGROW>
                tip_r = [tip_r, rf, 0.5, NaN, nring + 0.5, rt];        %#ok<AGROW>
            end
        else
            tip_t = [tip_t, t2];                                       %#ok<AGROW>
            tip_r = [tip_r, rf, rt];                                   %#ok<AGROW>
        end
    end

    % ================= heatmap =========================================
    mL = 90; mR = 110; mB = 80; mT = 30;
    axh = 22 * 19;
    fig = figure('Position', [40 40 AX_W + mL + mR, axh + mB + mT], 'Color', [1 1 1]);
    ax = axes('Parent', fig, 'Units', 'pixels', 'Position', [mL mB AX_W axh]);
    M = T.dcc(:, order)';
    vmax = max(M(:));
    imagesc(ax, x, 1:19, M, [0, vmax]);
    set(ax, 'YDir', 'reverse');
    hold(ax, 'on');
    colormap(fig, sdu_hex_cmap());
    plot(ax, tip_t, tip_r, '-', 'Color', [0 0 0], 'LineWidth', 1.6);
    plot(ax, [0 x_stop], [nring + 0.5, nring + 0.5], '-', 'Color', [1 1 1], 'LineWidth', 2);
    set(ax, 'YTick', 1:19, 'YTickLabel', LAB(order), 'FontName', 'Helvetica', ...
        'FontSize', 11, 'XTick', 0:10:ceil(x_stop), 'Layer', 'top', 'Box', 'on');
    xlim(ax, [0 x_stop]); ylim(ax, [0.5 19.5]);
    xlabel(ax, sprintf('time (s)   [%s, out and back]', strjoin(T.path_out, ' > ')), ...
        'FontName', 'Helvetica', 'FontSize', 14, 'Interpreter', 'none');
    ylabel(ax, 'cell (ring first, in path order)', 'FontName', 'Helvetica', 'FontSize', 13);
    cb = colorbar(ax, 'Units', 'pixels', 'Position', [mL + AX_W + 20, mB, 20, axh]);
    try, set(cb, 'FontName', 'Helvetica', 'FontSize', 10); catch, end
    ylabel(cb, 'dC/C0', 'Interpreter', 'none');
    save_fig(fig, fullfile(RESULTS_DIR, sprintf('%s_heatmap', tag)));
    if ~KEEP_FIGURES_OPEN, close(fig); end

    % ================= traces ==========================================
    cmap_ring = hsv(nring) * 0.75 + 0.1;            % cyclic: the path is a loop
    allv = T.dcc(:, k_ring); yhi = max(allv(:)) * 1.12; ylo = 0;
    axh = round(AX_W / 5);
    mL = 110; mR = 40; mB = 90; mT = 40;
    fig = figure('Position', [40 40 AX_W + mL + mR, axh + mB + mT], 'Color', [1 1 1]);
    ax = axes('Parent', fig, 'Units', 'pixels', 'Position', [mL mB AX_W axh]);
    hold(ax, 'on');
    fill([0 T_PRE T_PRE 0], [ylo ylo yhi yhi], [0.918 0.918 0.918], 'Parent', ax, 'EdgeColor', 'none');
    for j = 1:numel(T.blocks)
        B = T.blocks(j);
        if strcmp(B.phase, 'slide')
            if strcmp(B.dir, 'fwd'), rgb = SLIDE_OUT_RGB; else, rgb = SLIDE_BACK_RGB; end
        else
            rgb = HOLD_RGB;
        end
        xs = T_PRE + [B.t_start B.t_end];
        fill([xs(1) xs(2) xs(2) xs(1)], [ylo ylo yhi yhi], rgb, 'Parent', ax, 'EdgeColor', 'none');
    end
    for i = 1:nring
        plot(ax, x(:), T.dcc(:, k_ring(i)), '-', 'Color', cmap_ring(i, :), 'LineWidth', 1.8);
    end
    % pad names at the holds, in that pad's colour
    for j = 1:numel(T.blocks)
        B = T.blocks(j);
        if strcmp(B.phase, 'slide'), continue; end
        i = find(strcmp(ring, B.to), 1);
        text(ax, T_PRE + (B.t_start + B.t_end) / 2, ylo + 0.95 * (yhi - ylo), B.to, ...
            'Color', cmap_ring(i, :) * 0.8, 'FontWeight', 'bold', 'FontName', 'Helvetica', ...
            'FontSize', 11, 'HorizontalAlignment', 'center', 'Interpreter', 'none');
    end
    text(ax, T_PRE / 2, ylo + 0.95 * (yhi - ylo), ring{1}, 'Color', cmap_ring(1, :) * 0.8, ...
        'FontWeight', 'bold', 'FontName', 'Helvetica', 'FontSize', 11, ...
        'HorizontalAlignment', 'center', 'Interpreter', 'none');
    set(ax, 'FontName', 'Helvetica', 'FontSize', 14, 'Box', 'on', 'Layer', 'top', ...
        'XTick', 0:10:ceil(x_stop));
    xlim(ax, [0 x_stop]); ylim(ax, [ylo yhi]);
    xlabel(ax, 'time (s)   [blue: slide out, green: slide back, grey: hold]', ...
        'FontName', 'Helvetica', 'FontSize', 15);
    ylabel(ax, '\DeltaC/C_0', 'Interpreter', 'tex', 'FontName', 'Helvetica', 'FontSize', 17);
    save_fig(fig, fullfile(RESULTS_DIR, sprintf('%s_traces', tag)));
    if ~KEEP_FIGURES_OPEN, close(fig); end

    % ================= tracking at the holds ===========================
    H = T.holds;
    fid = fopen(fullfile(RESULTS_DIR, sprintf('%s_tracking.csv', tag)), 'w');
    fprintf(fid, 'pass,direction,pad,hottest_cell,dcc_pad,dcc_hottest,hit\n');
    hit_rate = nan(nring, 2);
    for i = 1:nring
        for dI = 1:2
            dstr = {'fwd', 'rev'}; dstr = dstr{dI};
            n_h = 0; n_hit = 0;
            for h = 1:numel(H)
                if ~strcmp(H(h).pad, ring{i}) || ~strcmp(H(h).dir, dstr), continue; end
                n_h = n_h + 1;
                [~, kb] = max(H(h).dcc);
                n_hit = n_hit + (kb == k_ring(i));
            end
            if n_h > 0, hit_rate(i, dI) = 100 * n_hit / n_h; end
        end
    end
    n_hit_all = 0;
    for h = 1:numel(H)
        kp = find(strcmp(LAB, H(h).pad), 1);
        [vb, kb] = max(H(h).dcc);
        n_hit_all = n_hit_all + (kb == kp);
        fprintf(fid, '%d,%s,%s,%s,%.4f,%.4f,%d\n', H(h).pass, H(h).dir, H(h).pad, ...
            LAB{kb}, H(h).dcc(kp), vb, kb == kp);
    end
    fclose(fid);
    fprintf('  tracking: hottest cell == pad under tip in %d/%d holds (%.0f%%)\n', ...
        n_hit_all, numel(H), 100 * n_hit_all / numel(H));

    fig = figure('Position', [40 40 140 + 70 * nring, 460], 'Color', [1 1 1]);
    ax = axes('Parent', fig);
    hb = bar(ax, 1:nring, hit_rate, 'grouped');
    try
        set(hb(1), 'FaceColor', [0.4784 0.3765 0.2510]);
        set(hb(2), 'FaceColor', [0.8784 0.4941 0.2353]);
    catch
    end
    set(ax, 'XTick', 1:nring, 'XTickLabel', ring, 'FontName', 'Helvetica', ...
        'FontSize', 13, 'YLim', [0 105], 'Box', 'on');
    ylabel(ax, 'hottest cell = pad (%)', 'FontName', 'Helvetica', 'FontSize', 14);
    xlabel(ax, 'pad under the tip (path order)', 'FontName', 'Helvetica', 'FontSize', 14);
    legend(ax, {'out', 'back'}, 'Location', 'southoutside', 'Orientation', 'horizontal');
    save_fig(fig, fullfile(RESULTS_DIR, sprintf('%s_tracking', tag)));
    if ~KEEP_FIGURES_OPEN, close(fig); end
    fprintf('  done -> %s\n', RESULTS_DIR);
end
fprintf('\nAll sessions done.\n');
