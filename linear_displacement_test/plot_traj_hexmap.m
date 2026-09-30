% plot_traj_hexmap.m — Multi-point trajectory | hexmap strips + animation
% =========================================================================
% For logs from trajectory_displacement.py. Values from traj_timeline.m
% (session locate baseline, piecewise-aligned median across the passes).
% Two outputs per session, in results/trajectory/hexmap/:
%
%   <tag>_strip.(png|svg) -- the honeycomb at the middle of every HOLD:
%                            top row the pads on the way out, bottom row
%                            the pads on the way back. The pad under the tip
%                            is ringed; if the sensor tracks, it is also the
%                            hottest cell.
%   frames/<tag>/f###.png -- the whole pass animated on a time axis:
%                            honeycomb + tip on top, the ring-cell traces
%                            drawn progressively underneath. Assembled by
%   ./make_slide_videos.sh -d results/trajectory/hexmap
%
% Tip position moves between the pads' CANONICAL board coordinates, at
% constant speed within each slide.
% =========================================================================

clear; close all; clc;

HERE = fileparts(mfilename('fullpath'));
addpath(HERE);
addpath(fullfile(HERE, '..', 'Interdome_touch', 'matlab_analysis'));

%% ---- helpers (Octave requires these at the TOP) ------------------------
function v = dcc_at_time(T, t)
    v = nan(1, 19);
    x = T.t_centre(:);
    for k = 1:19
        y = T.dcc(:, k); ok = isfinite(y);
        if nnz(ok) < 2, continue; end
        v(k) = interp1(x(ok), y(ok), t, 'linear', 'extrap');
    end
end

function [xy, pad_now, arrow] = tip_at(T, t, CANON, LAB)
% Tip position at time t (s from the first slide), the pad it is resting on
% ('' while sliding), and the heading text.
    B = T.blocks;
    pos = @(p) CANON(find(strcmp(LAB, p), 1), :);
    pad_now = ''; arrow = '';
    if t < B(1).t_start
        xy = pos(B(1).from); pad_now = B(1).from; return;
    end
    for j = 1:numel(B)
        if t >= B(j).t_start && t < B(j).t_end
            if strcmp(B(j).phase, 'slide')
                f = (t - B(j).t_start) / (B(j).t_end - B(j).t_start);
                xy = pos(B(j).from) + f * (pos(B(j).to) - pos(B(j).from));
                arrow = sprintf('%s -> %s', B(j).from, B(j).to);
            else
                xy = pos(B(j).to); pad_now = B(j).to;
            end
            return;
        end
    end
    xy = pos(B(end).to); pad_now = B(end).to;
end

function draw_hex(ax, CANON, vals, tip_xy, ring_k, vmin, vmax, cmap, hex_r, xr, yr, filled)
    hold(ax, 'on');
    for k = 1:19
        [vx, vy] = hex_vertices(CANON(k, 1), CANON(k, 2), hex_r);
        fill(vx, vy, value_to_rgb(vals(k), vmin, vmax, cmap), 'Parent', ax, ...
            'EdgeColor', [0.35 0.35 0.35], 'LineWidth', 0.7);
    end
    if ~isempty(ring_k)   % outline the pad under the tip
        [vx, vy] = hex_vertices(CANON(ring_k, 1), CANON(ring_k, 2), hex_r * 0.93);
        plot(ax, [vx(:); vx(1)], [vy(:); vy(1)], '-', 'Color', [0 0 0], 'LineWidth', 2.2);
    end
    th = linspace(0, 2 * pi, 40); r = 1.6;
    cx = tip_xy(1) + r * cos(th); cy = tip_xy(2) + r * sin(th);
    if filled
        fill(cx, cy, [1 1 1], 'Parent', ax, 'EdgeColor', [0 0 0], 'LineWidth', 1.4);
    else
        plot(ax, cx, cy, '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.4);
    end
    xlim(ax, xr); ylim(ax, yr);
    set(ax, 'XTick', [], 'YTick', [], 'Box', 'on');
end

% ── CONFIG ───────────────────────────────────────────────────────────────
SESSIONS = { ...
    'hollow_dome_5_iterations_external_diameter_session_20260930_191043.csv', 'ring_external'; ...
    'hollow_dome_5_iterations_internal_diameter_session_20260930_193505.csv', 'ring_internal'};
T_PRE  = 6.0;
T_POST = 3.0;
BIN_S  = 0.5;
FRAMES_PER_S = 2;     % frames per second of DATA time (a pass is 2-4 min)
WRITE_ANIM_FRAMES = isempty(getenv('NO_ANIM'));
DEPTH_ZERO_MM = 5.0;
HEX_RADIUS = 8.0 / sqrt(3);
SLIDE_OUT_RGB  = [0.831 0.855 0.878];
SLIDE_BACK_RGB = [0.851 0.878 0.851];
HOLD_RGB       = [0.945 0.945 0.945];
CANON = [ -16,   0;  -12,   7;   -8,  14;  -12,  -7;   -8,   0;
           -4,   7;    0,  14;   -8, -14;   -4,  -7;    0,   0;
            4,   7;    8,  14;    0, -14;    4,  -7;    8,   0;
           12,   7;    8, -14;   12,  -7;   16,   0];
LAB = {'a1','a2','a3','b1','b2','b3','b4','c1','c2','c3', ...
       'c4','c5','d2','d3','d4','d5','e3','e4','e5'};
% animation layout (pixels), same scheme as plot_linear_hexmap_frames_v2.m
FIG_W = 800; FIG_H = 960;
AX_L  = 95;  AX_W = 591;
TR_B  = 85;  TR_H = 185;
HX_B  = 350;
HPAD = HEX_RADIUS * 1.2;
XR = [min(CANON(:, 1)) - HPAD, max(CANON(:, 1)) + HPAD];
YR = [min(CANON(:, 2)) - HPAD, max(CANON(:, 2)) + HPAD];
HX_H = round(AX_W * (YR(2) - YR(1)) / (XR(2) - XR(1)));

LOG_DIR     = fullfile(HERE, 'logs');
RESULTS_DIR = fullfile(HERE, 'results', 'trajectory', 'hexmap');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end

for si = 1:size(SESSIONS, 1)
    csv_name = SESSIONS{si, 1};
    tag      = SESSIONS{si, 2};
    csv_path = fullfile(LOG_DIR, csv_name);
    fprintf('\n%s\n  SESSION: %s\n%s\n', repmat('=', 1, 70), tag, repmat('=', 1, 70));
    if ~exist(csv_path, 'file')
        warning('plot_traj_hexmap:missing', 'skipping %s', csv_name);
        continue;
    end
    T = traj_timeline(csv_path, T_PRE, T_POST, BIN_S);
    ring = unique(T.path_out, 'stable'); nring = numel(ring);
    k_ring = cellfun(@(p) find(strcmp(LAB, p), 1), ring);
    in = T.t_centre >= 0 & T.t_centre <= T.t_core_end;
    vals_all = T.dcc(in, :);
    vmin = 0; vmax = max(vals_all(:));
    cmap = sdu_hex_cmap();

    % ================= strip: one honeycomb per hold ===================
    hb = []; hdir = {};
    for j = 1:numel(T.blocks)
        if ~strcmp(T.blocks(j).phase, 'slide')
            hb(end + 1) = j; hdir{end + 1} = T.blocks(j).dir;   %#ok<SAGROW>
        end
    end
    out_cols  = [0, hb(strcmp(hdir, 'fwd'))];   % 0 = the opening hold at P1
    back_cols = hb(strcmp(hdir, 'rev'));
    ncol = max(numel(out_cols), numel(back_cols) + 1);
    fig = figure('Position', [30 30 200 * ncol + 160, 520], 'Color', [1 1 1]);
    mL = 0.05; mR = 0.07; mT = 0.07; mB = 0.02; gx = 0.004; gy = 0.08;
    pw = (1 - mL - mR - (ncol - 1) * gx) / ncol;
    ph = (1 - mT - mB - gy) / 2;
    last_ax = [];
    for row = 1:2
        yb = mB + (2 - row) * (ph + gy);
        if row == 1
            cols = out_cols; rl = 'out';
        else
            cols = [NaN, back_cols]; rl = 'back';   % back row aligned under the turnaround
        end
        for c = 1:numel(cols)
            if isnan(cols(c)), continue; end
            if cols(c) == 0
                t = -0.5; pad = ring{1};                % middle of hold_start
            else
                B = T.blocks(cols(c)); t = (B.t_start + B.t_end) / 2; pad = B.to;
            end
            ax = axes('Parent', fig, 'Position', [mL + (c - 1) * (pw + gx), yb, pw, ph - 0.05]);
            kp = find(strcmp(LAB, pad), 1);
            draw_hex(ax, CANON, dcc_at_time(T, t), CANON(kp, :), kp, vmin, vmax, ...
                cmap, HEX_RADIUS, XR, YR, true);
            axis(ax, 'equal'); xlim(ax, XR); ylim(ax, YR);
            [~, kh] = max(dcc_at_time(T, t));
            if kh == kp, tc = [0 0 0]; else, tc = [0.75 0.15 0.10]; end
            title(ax, sprintf('%s (hot %s)', pad, LAB{kh}), 'FontName', 'Helvetica', ...
                'FontSize', 10, 'Color', tc, 'Interpreter', 'none');
            last_ax = ax;
        end
        axl = axes('Parent', fig, 'Position', [0 yb mL ph - 0.05], 'Visible', 'off');
        text(axl, 0.5, 0.5, rl, 'Units', 'normalized', 'Rotation', 90, ...
            'HorizontalAlignment', 'center', 'FontName', 'Helvetica', ...
            'FontSize', 14, 'FontWeight', 'bold');
    end
    colormap(fig, cmap); caxis(last_ax, [vmin vmax]);
    cb = colorbar(last_ax, 'Position', [1 - mR + 0.015, mB, 0.012, 1 - mT - mB]);
    try, set(cb, 'FontName', 'Helvetica', 'FontSize', 9); catch, end
    ylabel(cb, 'dC/C0', 'Interpreter', 'none');
    save_fig(fig, fullfile(RESULTS_DIR, sprintf('%s_strip', tag)));
    close(fig);

    % ================= animation =======================================
    if ~WRITE_ANIM_FRAMES, continue; end
    anim_dir = fullfile(RESULTS_DIR, 'frames', tag);
    if ~exist(anim_dir, 'dir'), mkdir(anim_dir); end
    x = T.t_centre(:) + T_PRE;
    x_stop = T.t_stop + T_PRE;
    cmap_ring = hsv(nring) * 0.75 + 0.1;
    ally = T.dcc(:, k_ring); ylo = 0; yhi = max(ally(:)) * 1.10;
    n_anim = round(x_stop * FRAMES_PER_S) + 1;
    tt = linspace(0, x_stop, n_anim);
    for b = 1:n_anim
        t = tt(b); tr = t - T_PRE;          % time relative to first slide
        [tip, pad_now, arrow] = tip_at(T, tr, CANON, LAB);
        in_contact = tr >= T.t_engage && tr <= T.t_core_end;
        if isempty(pad_now), kp = []; else, kp = find(strcmp(LAB, pad_now), 1); end

        f2 = figure('Position', [40 40 FIG_W FIG_H], 'Color', [1 1 1], 'Visible', 'off');
        axH = axes('Parent', f2, 'Units', 'pixels', 'Position', [AX_L HX_B AX_W HX_H]);
        draw_hex(axH, CANON, dcc_at_time(T, tr), tip, kp, vmin, vmax, cmap, ...
            HEX_RADIUS, XR, YR, in_contact);
        colormap(f2, cmap); caxis(axH, [vmin vmax]);
        cb = colorbar(axH, 'Units', 'pixels', 'Position', [AX_L + AX_W + 18, HX_B, 22, HX_H]);
        try, set(cb, 'FontName', 'Helvetica', 'FontSize', 10); catch, end

        axR = axes('Parent', f2, 'Units', 'pixels', 'Position', [AX_L TR_B AX_W TR_H]);
        hold(axR, 'on');
        fill([0 T_PRE T_PRE 0], [ylo ylo yhi yhi], [0.918 0.918 0.918], 'Parent', axR, 'EdgeColor', 'none');
        for j = 1:numel(T.blocks)
            B = T.blocks(j);
            if strcmp(B.phase, 'slide')
                if strcmp(B.dir, 'fwd'), rgb = SLIDE_OUT_RGB; else, rgb = SLIDE_BACK_RGB; end
            else
                rgb = HOLD_RGB;
            end
            xs = T_PRE + [B.t_start B.t_end];
            fill([xs(1) xs(2) xs(2) xs(1)], [ylo ylo yhi yhi], rgb, 'Parent', axR, 'EdgeColor', 'none');
        end
        drawn = x <= t;
        if nnz(drawn) >= 2
            for i = 1:nring
                plot(axR, x(drawn), T.dcc(drawn, k_ring(i)), '-', 'Color', cmap_ring(i, :), 'LineWidth', 1.6);
            end
        end
        plot(axR, [t t], [ylo yhi], '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 2.4);
        set(axR, 'FontName', 'Helvetica', 'FontSize', 12, 'Box', 'on', ...
            'XTick', 0:20:ceil(x_stop));
        xlim(axR, [0 x_stop]); ylim(axR, [ylo yhi]);
        xlabel(axR, 'time (s)', 'FontName', 'Helvetica', 'FontSize', 13);
        ylabel(axR, '\DeltaC/C_0', 'Interpreter', 'tex', 'FontName', 'Helvetica', 'FontSize', 13);
        % ring legend: pad names in their trace colours along the top
        for i = 1:nring
            text(axR, x_stop * (0.02 + 0.96 * (i - 0.5) / nring), ylo + 0.90 * (yhi - ylo), ring{i}, ...
                'Color', cmap_ring(i, :) * 0.8, 'FontWeight', 'bold', 'FontName', 'Helvetica', ...
                'FontSize', 10, 'HorizontalAlignment', 'center', 'Interpreter', 'none');
        end

        if isempty(arrow)
            if isempty(pad_now), head = ''; else, head = sprintf('at %s', pad_now); end
        else
            head = arrow;
        end
        if tr > T.t_core_end
            head = 'lift-off';
        elseif ~in_contact
            head = sprintf('approach %s', ring{1});
        end
        axh = axes('Parent', f2, 'Position', [0 0 1 1], 'Visible', 'off');
        text(axh, 0.45, 0.995, sprintf('%s    t = %.1f s', head, t), ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'top', 'FontWeight', 'bold', ...
            'FontName', 'Helvetica', 'FontSize', 15, 'Interpreter', 'none');
        text(axh, 0.45, 0.966, sprintf('%s    speed = %g mm/s    depth = %g mm', ...
                strjoin(T.path_out, '>'), T.speed_mm_s, T.depth_mm - DEPTH_ZERO_MM), ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'top', 'Color', [0.35 0.35 0.35], ...
            'FontName', 'Helvetica', 'FontSize', 10, 'Interpreter', 'none');
        print(f2, fullfile(anim_dir, sprintf('f%03d.png', b)), '-dpng', '-r100');
        close(f2);
    end
    fprintf('    %s: %d frames -> %s\n', tag, n_anim, anim_dir);
end
fprintf('\nAll sessions done.\n');
fprintf('\nTo build the videos:\n  ./make_slide_videos.sh -d results/trajectory/hexmap\n');
