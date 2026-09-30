% plot_bidir_hexmap_frames.m — Bidirectional slide | hexmap snapshots along the path
% =========================================================================
% The bidirectional counterpart of plot_linear_hexmap_frames.m, for logs
% from bidirectional_displacement.py. For each c3 <-> X segment the
% honeycomb is rendered at points along BOTH slides, cells filled by
% dC/C0, with a marker at the tip:
%
%   *_frames.(png|svg)    -- two-row strip: top row the slide OUT
%                            (c3 -> X, 0..100%), bottom row the slide BACK
%                            (X -> c3, 0..100%)
%   frames/<seg>/f###.png -- N_ANIM frames out then N_ANIM back, assembled
%                            by ./make_slide_videos.sh
%
% Values come from bidir_slide_timeline.m (segment-level locate baseline,
% median across the 5 passes). The logger's 'progress' column restarts on
% the return slide, so position is taken from TIME within each slide
% instead -- the speed is constant, so the two are proportional.
% =========================================================================

clear; close all; clc;

HERE = fileparts(mfilename('fullpath'));
addpath(HERE);
addpath(fullfile(HERE, '..', 'Interdome_touch', 'matlab_analysis'));

%% ---- helpers (defined up top -- Octave requires that) ------------------
function v = dcc_at_time(T_e, t)
% The 19 cell values at time t (s from outward-slide start), linearly
% interpolated between the measured bin centres.
    v = nan(1, 19);
    x = T_e.t_centre(:);
    for k = 1:19
        y = T_e.dcc(:, k);
        ok = isfinite(y);
        if nnz(ok) < 2, continue; end
        v(k) = interp1(x(ok), y(ok), t, 'linear', 'extrap');
    end
end

function draw_frame(ax, CANON, vals, tip_xy, vmin, vmax, cmap, hex_r)
% Honeycomb filled by dC/C0 + a marker at the tip.
    hold(ax, 'on');
    for k = 1:19
        [vx, vy] = hex_vertices(CANON(k, 1), CANON(k, 2), hex_r);
        fill(vx, vy, value_to_rgb(vals(k), vmin, vmax, cmap), 'Parent', ax, ...
            'EdgeColor', [0.35 0.35 0.35], 'LineWidth', 0.7);
    end
    th = linspace(0, 2 * pi, 40);
    r = 1.6;
    fill(tip_xy(1) + r * cos(th), tip_xy(2) + r * sin(th), [1 1 1], ...
        'Parent', ax, 'EdgeColor', [0 0 0], 'LineWidth', 1.4);
    axis(ax, 'equal');
    pad = hex_r * 1.2;
    xlim(ax, [min(CANON(:, 1)) - pad, max(CANON(:, 1)) + pad]);
    ylim(ax, [min(CANON(:, 2)) - pad, max(CANON(:, 2)) + pad]);
    set(ax, 'XTick', [], 'YTick', [], 'Box', 'on');
end

function [t, tip, lbl] = slide_state(T_e, dir_i, frac, p_from, p_to)
% Time, tip position and heading for fraction 'frac' along the slide OUT
% (dir_i = 1) or BACK (dir_i = 2).
    m = T_e.marks;
    if dir_i == 1
        t = frac * m.fwd_end;
        tip = p_from + frac * (p_to - p_from);
        lbl = sprintf('%s -> %s  (out)', T_e.from_label, T_e.to_label);
    else
        t = m.rev_start + frac * (m.rev_end - m.rev_start);
        tip = p_to + frac * (p_from - p_to);
        lbl = sprintf('%s -> %s  (back)', T_e.to_label, T_e.from_label);
    end
end

% ── CONFIG ───────────────────────────────────────────────────────────────
SESSIONS = { ...
    'hollow_dome_5_iterations_c3_bidirectional_allpoints_session_20260930_160646.csv', 'c3_bidir'};

T_PRE  = 6.0;
T_POST = 3.0;
BIN_S  = 0.25;
STRIP_PCT = [0 20 40 60 80 100];
N_ANIM    = 61;      % per direction, so 122 frames per segment
WRITE_ANIM_FRAMES = isempty(getenv("NO_ANIM"));
% Re-render only these '<tag>_<from>_to_<to>' segments; {} = all, e.g.
%   ONLY_SEGMENTS='c3_bidir_c3_to_a1' octave plot_bidir_hexmap_frames.m
ONLY_SEGMENTS = {};
env_only = getenv('ONLY_SEGMENTS');
if ~isempty(env_only)
    ONLY_SEGMENTS = strsplit(env_only, ',');
    fprintf('ONLY_SEGMENTS from env: %s\n', env_only);
end
DEPTH_ZERO_MM = 5.0;   % logged depth is measured 5 mm above the surface
HEX_RADIUS = 8.0 / sqrt(3);
KEEP_FIGURES_OPEN = false;

CANON = [ -16,   0;  -12,   7;   -8,  14;  -12,  -7;   -8,   0;
           -4,   7;    0,  14;   -8, -14;   -4,  -7;    0,   0;
            4,   7;    8,  14;    0, -14;    4,  -7;    8,   0;
           12,   7;    8, -14;   12,  -7;   16,   0];
LAB = {'a1','a2','a3','b1','b2','b3','b4','c1','c2','c3', ...
       'c4','c5','d2','d3','d4','d5','e3','e4','e5'};

LOG_DIR     = fullfile(HERE, 'logs');
RESULTS_DIR = fullfile(HERE, 'results', 'bidirectional', 'hexmap_frames');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end

for si = 1:size(SESSIONS, 1)
    csv_name = SESSIONS{si, 1};
    tag      = SESSIONS{si, 2};
    csv_path = fullfile(LOG_DIR, csv_name);
    fprintf('\n%s\n  SESSION: %s\n%s\n', repmat('=', 1, 70), tag, repmat('=', 1, 70));
    if ~exist(csv_path, 'file')
        warning('plot_bidir_hexmap_frames:missing', 'skipping %s', csv_name);
        continue;
    end

    S = bidir_slide_timeline(csv_path, T_PRE, T_POST, BIN_S);
    % shared colour scale over the two slides and the hold between them
    allv = [];
    for e = 1:numel(S)
        in = S(e).t_centre >= 0 & S(e).t_centre <= S(e).marks.rev_end;
        v = S(e).dcc(in, :); allv = [allv; v(isfinite(v))];   %#ok<AGROW>
    end
    vmin = min(allv); vmax = max(allv);
    cmap = sdu_hex_cmap();
    fprintf('  %d segments | shared dC/C0 scale [%.3f, %.3f]\n', numel(S), vmin, vmax);

    for e = 1:numel(S)
        p_from = CANON(find(strcmp(LAB, S(e).from_label), 1), :);
        p_to   = CANON(find(strcmp(LAB, S(e).to_label), 1), :);
        seg = sprintf('%s_to_%s', S(e).from_label, S(e).to_label);
        if ~isempty(ONLY_SEGMENTS) && ...
                ~any(strcmp(ONLY_SEGMENTS, sprintf('%s_%s', tag, seg)))
            continue;
        end

        % ---- two-row strip ----------------------------------------------
        n_strip = numel(STRIP_PCT);
        fig = figure('Position', [50 50 300 * n_strip + 150, 760], 'Color', [1 1 1]);
        mL = 0.06; mR = 0.10; mT = 0.05; mB = 0.02; gx = 0.008; gy = 0.06;
        pw = (1 - mL - mR - (n_strip - 1) * gx) / n_strip;
        ph = (1 - mT - mB - gy) / 2;
        last_ax = [];
        for dir_i = 1:2
            yb = mB + (2 - dir_i) * (ph + gy);
            for fI = 1:n_strip
                [t, tip] = slide_state(S(e), dir_i, STRIP_PCT(fI) / 100, p_from, p_to);
                ax = axes('Parent', fig, 'Position', [mL + (fI - 1) * (pw + gx), yb, pw, ph - 0.04]);
                draw_frame(ax, CANON, dcc_at_time(S(e), t), tip, vmin, vmax, cmap, HEX_RADIUS);
                title(ax, sprintf('%d%%', STRIP_PCT(fI)), 'FontName', 'Helvetica', 'FontSize', 13);
                last_ax = ax;
            end
            % row label on the left
            axl = axes('Parent', fig, 'Position', [0 yb mL ph - 0.04], 'Visible', 'off');
            if dir_i == 1, rl = 'out'; else, rl = 'back'; end
            text(axl, 0.5, 0.5, rl, 'Units', 'normalized', 'Rotation', 90, ...
                'HorizontalAlignment', 'center', 'FontName', 'Helvetica', ...
                'FontSize', 15, 'FontWeight', 'bold', 'Interpreter', 'none');
        end
        colormap(fig, cmap); caxis(last_ax, [vmin, vmax]);
        cb = colorbar(last_ax, 'Position', [1 - mR + 0.025, mB, 0.014, 1 - mT - mB]);
        try, set(cb, 'FontName', 'Helvetica', 'FontSize', 9); catch, end
        ylabel(cb, 'dC/C0', 'Interpreter', 'none');
        save_fig(fig, fullfile(RESULTS_DIR, sprintf('%s_%s_frames', tag, seg)));
        if ~KEEP_FIGURES_OPEN, close(fig); end

        % ---- animation frames: out, then back ---------------------------
        if WRITE_ANIM_FRAMES
            anim_dir = fullfile(RESULTS_DIR, 'frames', sprintf('%s_%s', tag, seg));
            if ~exist(anim_dir, 'dir'), mkdir(anim_dir); end
            tt = linspace(0, 1, N_ANIM);
            n = 0;
            for dir_i = 1:2
                for b = 1:N_ANIM
                    [t, tip, lbl] = slide_state(S(e), dir_i, tt(b), p_from, p_to);
                    f2 = figure('Position', [50 50 620 620], 'Color', [1 1 1], 'Visible', 'off');
                    ax2 = axes('Parent', f2, 'Position', [0.04 0.04 0.80 0.82]);
                    draw_frame(ax2, CANON, dcc_at_time(S(e), t), tip, vmin, vmax, cmap, HEX_RADIUS);
                    colormap(f2, cmap); caxis(ax2, [vmin, vmax]);
                    cb2 = colorbar(ax2, 'Position', [0.87 0.04 0.03 0.82]);
                    try, set(cb2, 'FontName', 'Helvetica', 'FontSize', 9); catch, end
                    axh = axes('Parent', f2, 'Position', [0 0 1 1], 'Visible', 'off');
                    text(axh, 0.44, 0.985, sprintf('%s    %.0f%%', lbl, 100 * tt(b)), ...
                        'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                        'VerticalAlignment', 'top', 'FontWeight', 'bold', ...
                        'FontName', 'Helvetica', 'FontSize', 14, 'Interpreter', 'none');
                    text(axh, 0.44, 0.940, sprintf('speed = %g mm/s    depth = %g mm', ...
                            S(e).speed_mm_s, S(e).depth_mm - DEPTH_ZERO_MM), ...
                        'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                        'VerticalAlignment', 'top', 'Color', [0.35 0.35 0.35], ...
                        'FontName', 'Helvetica', 'FontSize', 11, 'Interpreter', 'none');
                    n = n + 1;
                    print(f2, fullfile(anim_dir, sprintf('f%03d.png', n)), '-dpng', '-r110');
                    close(f2);
                end
            end
            fprintf('    %s: %d frames -> %s\n', seg, n, anim_dir);
        end
    end
    fprintf('  done -> %s\n', RESULTS_DIR);
end

fprintf('\nAll sessions done.\n');
fprintf(['\nTo build the videos:\n' ...
         '  ./make_slide_videos.sh -d results/bidirectional/hexmap_frames -o c3_bidir -c\n']);
