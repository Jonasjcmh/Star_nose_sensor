% plot_bidir_response.m — Bidirectional slide | cell response over a round trip
% =========================================================================
% The bidirectional counterpart of plot_linear_response.m, for logs from
% bidirectional_displacement.py. For each c3 <-> X segment, the START cell
% (c3) and the DESTINATION cell are plotted as dC/C0 against time over one
% full round trip:
%
%   approach + press down | slide out | hold | slide back | hold + release
%
% so the response hands off c3 -> X on the way out and X -> c3 on the way
% back. Median across the 5 passes, with a q1-q3 band.
%
% Values come from bidir_slide_timeline.m -- see there for the baseline
% (the segment's single pre-contact 'locate' reading, since the tip stays
% engaged between passes) and for which passes feed which part of the
% window (approach: pass 0 only; lift-off: last pass only; the rest: all).
%
% TIME ORIGIN is the start of the plotted window; the outward slide starts
% at exactly T_PRE (6 s), as in the one-way figures.
%
% Output: results/bidirectional/response_vs_position/<tag>_slide_<from>_to_<to>.(png|svg)
% =========================================================================

clear; close all; clc;

HERE = fileparts(mfilename('fullpath'));
addpath(HERE);
addpath(fullfile(HERE, '..', 'Interdome_touch', 'matlab_analysis'));

% ── CONFIG ───────────────────────────────────────────────────────────────
SESSIONS = { ...
    'hollow_dome_5_iterations_c3_bidirectional_allpoints_session_20260930_160646.csv', 'c3_bidir'};

T_PRE  = 6.0;    % s before the outward slide (reaches back into 'locate')
T_POST = 3.0;    % s after the return hold (covers the lift-off)
BIN_S  = 0.25;
LAB = {'a1','a2','a3','b1','b2','b3','b4','c1','c2','c3', ...
       'c4','c5','d2','d3','d4','d5','e3','e4','e5'};
START_RGB = [0.8784 0.4941 0.2353];   % SDU orange - start cell
END_RGB   = [0.4784 0.3765 0.2510];   % SDU brown  - destination cell
AX_FONT   = 20;
TICK_FONT = 18;
% approach+press | slide out | hold | slide back | hold + release
STAGE_RGB = [0.918 0.918 0.918;
             0.831 0.855 0.878;
             0.945 0.945 0.945;
             0.851 0.878 0.851;   % return slide: a green tint, so the two
             0.945 0.945 0.945];  % directions read apart at a glance
STAGE_NAMES = {'approach + press down', 'slide out', 'hold', ...
               'slide back', 'hold + release'};
PANEL_ASPECT_RATIO = 4.0;
AX_WIDTH_PX        = 1280;
KEEP_FIGURES_OPEN  = false;

LOG_DIR     = fullfile(HERE, 'logs');
RESULTS_DIR = fullfile(HERE, 'results', 'bidirectional', 'response_vs_position');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end

for si = 1:size(SESSIONS, 1)
    csv_name = SESSIONS{si, 1};
    tag      = SESSIONS{si, 2};
    csv_path = fullfile(LOG_DIR, csv_name);
    fprintf('\n%s\n  SESSION: %s\n%s\n', repmat('=', 1, 70), tag, repmat('=', 1, 70));
    if ~exist(csv_path, 'file')
        warning('plot_bidir_response:missing', 'skipping %s', csv_name);
        continue;
    end

    S = bidir_slide_timeline(csv_path, T_PRE, T_POST, BIN_S);
    fprintf('  %d segments, %d passes each\n', numel(S), S(1).n_pass);

    % one y-scale for the session, from the two plotted cells
    allv = [];
    for e = 1:numel(S)
        kk = [find(strcmp(LAB, S(e).from_label), 1), find(strcmp(LAB, S(e).to_label), 1)];
        v = [reshape(S(e).dcc(:, kk), [], 1); reshape(S(e).dcc_q1(:, kk), [], 1); ...
             reshape(S(e).dcc_q3(:, kk), [], 1)];
        allv = [allv; v(isfinite(v))];   %#ok<AGROW>
    end
    ylo = 0;                      % axis only -- see plot_linear_response.m
    yhi = max(allv) * 1.10;
    fprintf('  shared y-scale: [%.3f, %.3f]\n', ylo, yhi);

    for e = 1:numel(S)
        k_start = find(strcmp(LAB, S(e).from_label), 1);
        k_end   = find(strcmp(LAB, S(e).to_label), 1);
        m = S(e).marks;
        tshift = T_PRE;
        b_stop = tshift + S(e).t_stop;
        bounds = [0, tshift, tshift + m.fwd_end, tshift + m.rev_start, ...
                  tshift + m.rev_end, b_stop];

        mL = 130; mR = 40; mB = 105; mT = 22;
        axw = AX_WIDTH_PX; axh = round(AX_WIDTH_PX / PANEL_ASPECT_RATIO);
        fig = figure('Position', [70 70 axw + mL + mR, axh + mB + mT], 'Color', [1 1 1]);
        ax = axes('Parent', fig, 'Units', 'pixels', 'Position', [mL mB axw axh]);
        hold(ax, 'on');

        % stage shading -- fill(), not patch() (patch is blank in gnuplot)
        for sI = 1:5
            fill([bounds(sI) bounds(sI + 1) bounds(sI + 1) bounds(sI)], ...
                 [ylo ylo yhi yhi], STAGE_RGB(sI, :), 'Parent', ax, 'EdgeColor', 'none');
        end
        for sI = 2:5
            plot(ax, [bounds(sI) bounds(sI)], [ylo yhi], '-', ...
                'Color', [0.62 0.62 0.62], 'LineWidth', 0.8);
        end
        plot(ax, [0 b_stop], [0 0], '-', 'Color', [0.75 0.75 0.75]);

        x = S(e).t_centre + tshift;
        for kk = [k_start, k_end]
            if kk == k_start, rgb = START_RGB; else, rgb = END_RGB; end
            lo = S(e).dcc_q1(:, kk)'; hi = S(e).dcc_q3(:, kk)';
            ok = isfinite(lo) & isfinite(hi);
            if any(ok)
                xb = x(ok);
                fill([xb, fliplr(xb)], [lo(ok), fliplr(hi(ok))], rgb, ...
                    'Parent', ax, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
            end
            plot(ax, x(:), S(e).dcc(:, kk), '-', 'Color', rgb, 'LineWidth', 2.4);
        end

        % stage names along the top; the 1 s holds are too narrow for text
        px_per_s = axw / b_stop;
        ytop = ylo + 0.94 * (yhi - ylo);
        for sI = 1:5
            if (bounds(sI + 1) - bounds(sI)) * px_per_s < 60, continue; end
            text(ax, (bounds(sI) + bounds(sI + 1)) / 2, ytop, STAGE_NAMES{sI}, ...
                'Color', [0.28 0.28 0.28], 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', 'FontName', 'Helvetica', ...
                'FontSize', 13, 'Interpreter', 'none');
        end

        set(ax, 'FontName', 'Helvetica', 'FontSize', TICK_FONT, 'Box', 'on', ...
            'XColor', [0 0 0], 'YColor', [0 0 0], 'Layer', 'top', ...
            'XTick', 0:2:ceil(b_stop));
        xlim(ax, [0, b_stop]); ylim(ax, [ylo, yhi]);
        xlabel(ax, sprintf('time (s)   [%s \\leftrightarrow %s]', ...
            S(e).from_label, S(e).to_label), 'Interpreter', 'tex', ...
            'FontName', 'Helvetica', 'FontSize', AX_FONT);
        ylabel(ax, '\DeltaC/C_0', 'Interpreter', 'tex', ...
            'FontName', 'Helvetica', 'FontSize', AX_FONT);

        % hand-drawn legend in the empty top-left (see plot_linear_response.m)
        lx0 = 0.02 * b_stop; lx1 = 0.08 * b_stop; ltx = 0.095 * b_stop;
        lys = [0.80, 0.68] * yhi;
        lcol = {START_RGB, END_RGB};
        ltxt = {sprintf('start cell %s', S(e).from_label), ...
                sprintf('end cell %s', S(e).to_label)};
        for lI = 1:2
            plot(ax, [lx0 lx1], [lys(lI) lys(lI)], '-', 'Color', lcol{lI}, 'LineWidth', 2.4);
            text(ax, ltx, lys(lI), ltxt{lI}, 'FontName', 'Helvetica', ...
                'FontSize', 13, 'VerticalAlignment', 'middle', ...
                'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);
        end

        save_fig(fig, fullfile(RESULTS_DIR, sprintf('%s_slide_%s_to_%s', ...
            tag, S(e).from_label, S(e).to_label)));
        if ~KEEP_FIGURES_OPEN, close(fig); end
    end
    fprintf('  done -> %s\n', RESULTS_DIR);
end

fprintf('\nAll sessions done.\n');
